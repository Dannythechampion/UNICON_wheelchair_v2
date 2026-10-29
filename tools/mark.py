#!/usr/bin/env python3
"""Mark an arrival (or any moment) during a field run.

    ~/mark.py D4_arrive

On 2026-10-16 a bare `rostopic pub -1 /field_mark ...` reached no bag at all:
pub exits before rosbag has connected to the new publisher. So this waits for
a subscriber before publishing, and also appends wall-clock time and the
current map / odometry position to ~/field_marks.log, which survives even if
nothing was recording. The black box started by start_wheelchair_localization.sh
records /field_mark.
"""

import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

LOG = os.path.expanduser("~/field_marks.log")


def latest(topic, msg_type, timeout_s=2.0):
    try:
        msg = rospy.wait_for_message(topic, msg_type, timeout=timeout_s)
    except rospy.ROSException:
        return None
    p = msg.pose.pose.position
    return "%.2f,%.2f" % (p.x, p.y)


def main():
    if len(sys.argv) != 2 or not sys.argv[1].strip():
        sys.stderr.write("usage: mark.py <label>\n")
        return 64
    label = sys.argv[1].strip()
    wall = time.strftime("%Y-%m-%d %H:%M:%S")

    rospy.init_node("field_mark", anonymous=True, disable_signals=True)
    pub = rospy.Publisher("/field_mark", String, queue_size=1, latch=True)

    map_xy = latest("/fast_lio_icp/pose", PoseWithCovarianceStamped)
    odom_xy = latest("/Odometry", Odometry)

    deadline = time.time() + 5.0
    while pub.get_num_connections() == 0 and time.time() < deadline:
        time.sleep(0.1)
    recorded = pub.get_num_connections() > 0
    stamp = rospy.Time.now().to_sec()
    pub.publish(String(data="%s %.3f" % (label, stamp)))
    time.sleep(0.5)

    line = "%s,%.3f,%s,map=%s,odom=%s,in_bag=%s\n" % (
        wall, stamp, label, map_xy or "-", odom_xy or "-",
        "yes" if recorded else "NO")
    with open(LOG, "a") as f:
        f.write(line)
    sys.stdout.write(line)
    if not recorded:
        sys.stderr.write("WARNING: nothing subscribed to /field_mark - "
                         "logged to %s only\n" % LOG)
    return 0


if __name__ == "__main__":
    sys.exit(main())
