#!/usr/bin/env python3
"""Non-driving ROS1 A* worker for Safety-owned static obstacle detours.

This node NEVER sends /cmd_vel or modifies files. Safety is responsible for
request validity, obstacle persistence, collision checking and path commit.
"""
import json
import os
import sys
import threading

import rospy
import rospkg
from std_msgs.msg import String
from std_srvs.srv import Trigger
from pid_occupancy_snapshot import SNAPSHOT_SERVICE

from ff_pid_core import Route

localization_scripts = os.path.join(
    rospkg.RosPack().get_path("static_livox_localization"), "scripts")
sys.path.insert(0, localization_scripts)
from replan_path import replan

ROUTES = ("A_to_D4", "A_to_D5", "D4_to_A", "D4_to_D5", "D5_to_A", "D5_to_D4")


class ReplanWorker:
    def __init__(self):
        rospy.init_node("pid_astar_planner")
        self.busy = False
        self.lock = threading.Lock()
        self.output = rospy.Publisher("/ff_pid/replan/candidate", String, queue_size=1)
        self.mask = None
        self.folder = os.path.join(
            os.path.dirname(os.path.dirname(
                os.path.realpath(rospkg.RosPack().get_path("pid")))),
            "routes", "route2_manual_20261031")
        rospy.Subscriber("/ff_pid/replan/request", String, self.on_request, queue_size=1)
        rospy.loginfo("Static A* worker ready; never publishes velocity")

    def on_request(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            route = data["route"]
            if route not in ROUTES or not isinstance(data["plan_id"], str):
                return
            if not isinstance(data["pose"], list) or len(data["pose"]) != 3:
                return
            with self.lock:
                if self.busy:
                    return
                self.busy = True
            thread = threading.Thread(target=self.compute, args=(data,), daemon=True)
            thread.start()
        except (ValueError, TypeError, KeyError):
            return

    def compute(self, request):
        result = {"plan_id": request["plan_id"], "route": request["route"],
                  "status": "FAILED", "error": "UNKNOWN"}
        try:
            from route_mask import RouteMask
            original = Route(os.path.join(
                self.folder, request["route"] + "_waypoints.json"))
            if self.mask is None:
                self.mask = RouteMask(os.path.join(self.folder, "common_drivable_mask.yaml"))
            # Snapshot acquisition runs in this non-driving worker thread.
            # Never block the Safety 10 Hz timer with a ROS service call.
            rospy.wait_for_service(SNAPSHOT_SERVICE, timeout=2.0)
            response = rospy.ServiceProxy(SNAPSHOT_SERVICE, Trigger)()
            if not response.success:
                raise ValueError("occupancy unavailable: " + response.message[:120])
            occupancy = json.loads(response.message)
            source_age = rospy.Time.now().to_sec() - float(occupancy["stamp"])
            if source_age < -0.2 or source_age > 0.8:
                raise ValueError("occupancy source stale")
            blocking_id = int(request["blocker_id"])
            metadata = replan(original, self.mask, request["pose"],
                              float(request["station"]), request["objects"],
                              occupancy=occupancy, blocking_id=blocking_id)
            result.update(status="OK", error=None, metadata=metadata,
                          occupancy=occupancy)
        except Exception as exc:
            result["error"] = str(exc)[:160]
            rospy.logwarn("A* plan failed %s: %s", request["plan_id"], exc)
        finally:
            self.output.publish(String(data=json.dumps(result, separators=(",", ":"),
                                                       allow_nan=False)))
            with self.lock:
                self.busy = False


if __name__ == "__main__":
    ReplanWorker()
    rospy.spin()
