#!/usr/bin/env python3
"""UNICON FF–PD ROS1 실행 노드.

/ff_pid/start=True 명령 전까지 주행하지 않습니다.
실제 /cmd_vel을 발행하므로 NUC wheel_cmd_tmp.py가 /ff_pid를 허용해야 합니다.
장애물 회피/정지 판단 노드는 구현하지 않았으며 인원이 있는 곳에서 사용하면 안 됩니다.
"""
import json
import math
import os
import sys
import threading
from collections import deque

import numpy as np
import rospy
import rosgraph
import rospkg
import tf.transformations as tft
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16MultiArray, String
from std_srvs.srv import SetBool, SetBoolResponse

# Catkin 개발 공간에서는 실제 실행 파일이 devel/lib/pid의 중계 스크립트일 수 있습니다.
# 먼저 pid 패키지의 scripts 경로에서 Core를 찾고, 설치 공간 실행도 지원합니다.
pid_scripts = os.path.join(rospkg.RosPack().get_path("pid"), "scripts")
if os.path.isdir(pid_scripts):
    sys.path.insert(0, pid_scripts)
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ff_pid_core as core_settings
from ff_pid_core import FFPD, Route

existing_scripts = os.path.join(
    rospkg.RosPack().get_path("static_livox_localization"), "scripts")
sys.path.insert(0, existing_scripts)
from body_frame import (pose_correction, reference_correction,
                        route_chair_centre, REFERENCE_CHAIR_CENTRE)
from route_assets import mask_image_path, route_content_sha256, sha256
from route_mask import RouteMask

# 짧은 이름으로 선택할 수 있는 수동 경로 여섯 개.
ROUTE_NAMES = (
    "A_to_D4", "A_to_D5", "D4_to_A",
    "D4_to_D5", "D5_to_A", "D5_to_D4",
)


def selected_route_and_mask(route_name):
    """팀 저장소에서 경로 이름에 대응하는 JSON과 공통 마스크를 찾습니다."""
    if route_name not in ROUTE_NAMES:
        raise rospy.ROSInitException(
            "잘못된 경로 이름: %s (가능: %s)" %
            (route_name, ", ".join(ROUTE_NAMES)))
    pid_dir = os.path.realpath(rospkg.RosPack().get_path("pid"))
    repo_dir = os.path.dirname(os.path.dirname(pid_dir))
    asset_dir = os.path.join(repo_dir, "routes", "route2_manual_20261031")
    return (os.path.join(asset_dir, route_name + "_waypoints.json"),
            os.path.join(asset_dir, "common_drivable_mask.yaml"))


def validate_route_mask_binding(metadata, mask_path):
    """Safety Band 없이 경로 내용 및 공통 마스크의 SHA-256만 검증합니다."""
    binding = metadata.get("asset_binding")
    if not isinstance(binding, dict):
        raise ValueError("경로에 asset_binding이 없습니다")
    if binding.get("route_content_sha256") != route_content_sha256(metadata):
        raise ValueError("경로 JSON 무결성 검사 실패")
    if binding.get("drivable_mask_yaml_sha256") != sha256(mask_path):
        raise ValueError("경로와 공통 마스크 YAML의 해시가 일치하지 않습니다")
    if binding.get("drivable_mask_sha256") != sha256(mask_image_path(mask_path)):
        raise ValueError("경로와 공통 마스크 이미지의 해시가 일치하지 않습니다")


AUTO_MODE = 65  # NUC 구동부에서 사용하는 자동 모드 값

# ======================= ROS 실행·안전 설정 =======================
# 아래 값은 ROS 노드의 실행 조건 및 정지 판단에만 사용합니다.
# Kp, Kd, FF, 바퀴 간격, 속도 제한 등의 기본값은 ff_pid_core.py 상단에 있습니다.
# 필요한 경우 ROS private 파라미터로 실행 시 값만 덮어쓸 수 있습니다.
GOAL_TOLERANCE_M = 0.40         # 목표점 도착 허용 거리 [m]
STOPPED_WHEEL_THRESHOLD_MPS = 0.06  # 정지로 간주할 바퀴 속도 [m/s]
STOPPED_REPORTS_TO_FINISH = 3   # 종료 전 연속 정지 확인 횟수
MAX_POSE_AGE_S = 0.50           # 허용 위치 데이터 수신 지연 [s]
MAX_WHEEL_AGE_S = 0.35          # 허용 바퀴 상태 수신 지연 [s]
MAX_DIAG_AGE_S = 5.0            # 허용 GICP 진단 수신 지연 [s]
CONTROL_PERIOD_S = 0.02        # ROS 제어 주기 [s] (50Hz)
PEER_CHECK_PERIOD_S = 1.0      # /cmd_vel 중복 발행자 확인 주기 [s]
# ================================================================

def read_wheel(data):
    """바퀴 상태 코드(C/W/S, 33 오프셋, 0.1km/h 단위)를 m/s로 변환합니다."""
    if len(data) < 6:
        raise ValueError("wheel_status too short")
    def decode(direction, magnitude):
        d = int(direction)
        if d not in (67, 87, 83):
            raise ValueError("invalid wheel direction")
        m = int(magnitude)
        if m < 33 or m > 127:
            raise ValueError("invalid wheel magnitude")
        if d == 83:
            return 0.0
        speed = (m - 33) / 10.0 / 3.6
        return speed if d == 67 else -speed
    return decode(data[2], data[3]), decode(data[4], data[5])


class FFNode:
    def __init__(self):
        rospy.init_node("ff_pid")  # /cmd_vel 발행자 이름은 반드시 /ff_pid
        if rospy.get_name() != "/ff_pid":
            raise rospy.ROSInitException("node name must be exactly /ff_pid")
        # 경로는 A_to_D4처럼 이름만 입력합니다. 모든 경로가 같은 마스크를 씁니다.
        route_name = str(rospy.get_param("~route"))
        route_path, mask_path = selected_route_and_mask(route_name)
        profile = str(rospy.get_param("~body_frame_profile", "builtin"))
        self.route = Route(route_path)
        metadata = self.route.metadata
        if metadata.get("reference_point") != REFERENCE_CHAIR_CENTRE:
            raise rospy.ROSInitException("only chair-centred routes are allowed")
        if "body_frame_profile" not in metadata:
            raise rospy.ROSInitException("route missing body_frame_profile")
        # Safety Band에는 접근하지 않습니다. 원본 경로/마스크의 연결만 검증합니다.
        validate_route_mask_binding(metadata, mask_path)
        self.mask = RouteMask(mask_path)
        # 경로의 바퀴 중심선과 직선 연결 구간이 마스크 밖으로 나가면 시작하지 않습니다.
        if not self.mask.contains_many(self.route.xy).all():
            raise rospy.ROSInitException("경로 Waypoint가 공통 마스크 밖에 있습니다")
        if not self.mask.paths_are_contained([self.route.xy]).all():
            raise rospy.ROSInitException("경로 구간이 공통 마스크 밖을 통과합니다")
        # 기존 경로 추종기와 같은 기준점 및 차체 좌표 보정을 적용합니다.
        self.pose_transform = pose_correction(
            profile, str(metadata["body_frame_profile"])) @ reference_correction(
            str(metadata["reference_point"]), route_chair_centre(metadata))
        # 제어 파라미터 기본값은 Core 상단에서만 관리합니다.
        # 실행 시 _kp:=0.4처럼 ROS private 파라미터를 지정하면 해당 값만 덮어씁니다.
        # ROS 파라미터는 노드 시작 시 한 번만 읽습니다(주행 중 실시간 변경 아님).
        self.core = FFPD(
            self.route,
            kp=float(rospy.get_param("~kp", core_settings.KP)),
            kd=float(rospy.get_param("~kd", core_settings.KD)),
            ki=float(rospy.get_param("~ki", core_settings.KI)),
            max_v=float(rospy.get_param("~max_v", core_settings.MAX_V_MPS)),
            max_w=float(rospy.get_param("~max_w", core_settings.MAX_W_RAD_S)),
            wheel_track=core_settings.WHEEL_TRACK_M,  # 구동부 0.46m와 항상 일치
            min_wheel=float(rospy.get_param("~min_wheel_mps", core_settings.MIN_WHEEL_MPS)),
            max_wheel=float(rospy.get_param("~max_wheel_mps", core_settings.MAX_WHEEL_MPS)),
            lateral_accel=float(rospy.get_param("~max_lateral_accel", core_settings.LATERAL_ACCEL_MPS2)),
            preview_m=float(rospy.get_param("~preview_m", core_settings.CURVATURE_PREVIEW_M)),
            cte_deadzone=float(rospy.get_param("~cte_deadzone_m", core_settings.CTE_DEADZONE_M)),
            d_deadzone=float(rospy.get_param("~d_deadzone_mps", core_settings.D_DEADZONE_MPS)),
            d_limit=float(rospy.get_param("~d_clamp_rad_s", core_settings.D_CLAMP_RAD_S)),
            prediction_horizon_s=float(rospy.get_param("~prediction_horizon_s", core_settings.PREDICTION_HORIZON_S)),
            prediction_measured_s=float(rospy.get_param("~prediction_measured_s", core_settings.PREDICTION_MEASURED_S)),
            prediction_step_s=float(rospy.get_param("~prediction_step_s", core_settings.PREDICTION_STEP_S)),
            prediction_delay_s=float(rospy.get_param("~prediction_delay_s", core_settings.PREDICTION_DELAY_S)),
        )
        self.lock = threading.RLock()
        self.enabled = False
        self.finishing = False
        self.external_hold = False
        self.stopped_reports = 0
        self.competing_sources = []
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", GOAL_TOLERANCE_M))
        self.stopped_wheel_threshold = float(rospy.get_param("~stopped_wheel_threshold", STOPPED_WHEEL_THRESHOLD_MPS))
        self.pose = None
        self.pose_received = None
        self.gicp_received = None
        self.gicp_raw = None
        self.gicp_odom_ref = None
        self.gicp_stamp = None
        self.pose_seq = 0
        self.odom_history = deque(maxlen=100)
        self.odom_latest = None
        self.odom_received = None
        self.pitch_deg = 0.0
        self.motion_policy = "STOP"
        self.policy_received = None
        self.obstacle_stopped = False
        self.resume_after_obstacle = False
        self.wheel = None
        self.wheel_pair = None
        self.wheel_received = None
        self.mode = None
        self.tracking = ""
        self.diag_received = None
        self.reason = "PAUSED"
        self.last_tick = None
        self.last_command_v = 0.0
        self.last_command_w = 0.0
        self.max_pose_age = float(rospy.get_param("~max_pose_age", MAX_POSE_AGE_S))
        self.max_wheel_age = float(rospy.get_param("~max_wheel_age", MAX_WHEEL_AGE_S))
        self.max_diag_age = float(rospy.get_param("~max_diag_age", MAX_DIAG_AGE_S))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status = rospy.Publisher("/ff_pid/status", String, queue_size=3)
        rospy.Subscriber("/fast_lio_icp/pose", PoseWithCovarianceStamped,
                         self.on_pose, queue_size=5)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/ff_pid/motion_policy", String,
                         self.on_motion_policy, queue_size=2)
        rospy.Subscriber("/wheel_status", Int16MultiArray,
                         self.on_wheel, queue_size=5)
        rospy.Subscriber("/fast_lio_icp/localization_diagnostics", DiagnosticArray,
                         self.on_diag, queue_size=5)
        # 외부 안전 감시 노드가 정지를 요청할 수 있습니다.
        # 단, 이 입력이 해제되더라도 자동으로 주행을 재개하지 않습니다.
        rospy.Subscriber("/ff_pid/stop_request", Bool, self.on_external_stop,
                         queue_size=2)
        rospy.Service("/ff_pid/start", SetBool, self.on_start)
        # ROS Master 조회가 지연될 수 있어 50Hz 제어 루프와 분리합니다.
        self.peer_timer = rospy.Timer(rospy.Duration(PEER_CHECK_PERIOD_S), self.on_peer_check)
        self.timer = rospy.Timer(rospy.Duration(CONTROL_PERIOD_S), self.on_tick)
        rospy.on_shutdown(self.stop)
        rospy.loginfo("FF-PD ready: %s, %d waypoints, %.1f m; PAUSED",
                      route_path, len(self.route.xy), self.route.length)

    def on_odom(self, msg):
        try:
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            transform = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            transform[:3, 3] = (p.x, p.y, p.z)
            _, pitch, _ = tft.euler_from_matrix(transform)
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or not math.isfinite(stamp):
                return
            if not np.isfinite(transform).all():
                return
            now = rospy.Time.now().to_sec()
            with self.lock:
                self.odom_history.append((stamp, transform))
                self.odom_latest = transform
                self.odom_received = now
                self.pitch_deg = math.degrees(pitch)
        except Exception as exc:
            rospy.logwarn_throttle(5, "bad odom: %s", exc)

    def on_pose(self, msg):
        try:
            if msg.header.frame_id not in ("map", "/map"):
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            raw_map = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            raw_map[:3, 3] = (p.x, p.y, p.z)
            now = rospy.Time.now().to_sec()
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or abs(now - stamp) > self.max_pose_age:
                return
            if not np.isfinite(raw_map).all():
                return
            with self.lock:
                nearest = min(self.odom_history, key=lambda e: abs(e[0] - stamp)) if self.odom_history else None
                odom_ref = nearest[1].copy() if nearest is not None and abs(nearest[0] - stamp) <= 0.10 else None
                self.gicp_raw = raw_map
                self.gicp_odom_ref = odom_ref
                self.gicp_stamp = stamp
                self.gicp_received = now
                self.pose_seq += 1
                matrix = raw_map @ self.pose_transform
                _, _, yaw = tft.euler_from_matrix(matrix)
                self.pose = (float(matrix[0, 3]), float(matrix[1, 3]), float(yaw))
                self.pose_received = now
        except Exception as exc:
            rospy.logwarn_throttle(5, "bad pose: %s", exc)

    def current_pose(self, now):
        if self.gicp_raw is None or self.gicp_received is None:
            return None
        if now - self.gicp_received > self.max_pose_age:
            return None
        raw_map = self.gicp_raw
        if (self.gicp_odom_ref is not None and self.odom_latest is not None and
                self.odom_received is not None and now - self.odom_received <= 0.25):
            raw_map = raw_map @ np.linalg.inv(self.gicp_odom_ref) @ self.odom_latest
        matrix = raw_map @ self.pose_transform
        _, _, yaw = tft.euler_from_matrix(matrix)
        return float(matrix[0, 3]), float(matrix[1, 3]), float(yaw)

    def on_motion_policy(self, msg):
        state = str(msg.data).strip().upper()
        if state not in ("CLEAR", "SLOW", "STOP"):
            rospy.logwarn_throttle(5, "Unknown obstacle motion policy %s", state)
            return
        with self.lock:
            self.motion_policy = state
            self.policy_received = rospy.Time.now().to_sec()
            if state == "STOP":
                if self.enabled:
                    self.resume_after_obstacle = True
                self.obstacle_stopped = True
                self.disable("OBSTACLE_STOP")
            elif self.obstacle_stopped and self.resume_after_obstacle and not self.external_hold:
                if self.readiness(rospy.Time.now().to_sec()) is None:
                    self.core.reset()
                    self.enabled = True
                    self.obstacle_stopped = False
                    self.resume_after_obstacle = False
                    self.publish_status("RUNNING")

    def on_wheel(self, msg):
        try:
            left, right = read_wheel(msg.data)
            v = (left + right)/2
            w = (right - left) / self.core.wheel_track
            mode = int(msg.data[1])
            with self.lock:
                self.wheel = (v, w)
                self.wheel_pair = (left, right)
                self.wheel_received = rospy.Time.now().to_sec()
                self.mode = mode
                if self.finishing:
                    if (abs(left) <= self.stopped_wheel_threshold and
                            abs(right) <= self.stopped_wheel_threshold):
                        self.stopped_reports += 1
                    else:
                        self.stopped_reports = 0
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(5, "invalid wheel_status: %s", exc)

    def on_diag(self, msg):
        for d in msg.status:
            if d.name == "fast_lio_icp":
                with self.lock:
                    self.tracking = d.message
                    self.diag_received = rospy.Time.now().to_sec()
                return

    def on_external_stop(self, msg):
        with self.lock:
            if msg.data:
                self.external_hold = True
                self.resume_after_obstacle = False
                self.disable("EXTERNAL_STOP_REQUEST")
            else:
                self.external_hold = False
                # 정지 요청 해제만으로 주행이 다시 시작되지 않게 합니다.

    def readiness(self, now):
        if self.policy_received is None or now - self.policy_received > 1.0:
            return "SAFETY_POLICY_STALE"
        if self.current_pose(now) is None:
            return "POSE_STALE"
        if self.wheel is None or self.wheel_received is None or now - self.wheel_received > self.max_wheel_age:
            return "WHEEL_STALE"
        if self.diag_received is None or now - self.diag_received > self.max_diag_age or self.tracking != "TRACKING":
            return "NO_TRACKING"
        if self.mode != AUTO_MODE:
            return "NOT_AUTO_MODE"
        x, y, _ = self.current_pose(now)
        if not self.mask.contains((x, y)):
            return "OUTSIDE_SAFE_ROUTE"
        return None

    def competing_publishers(self):
        """다른 노드가 /cmd_vel을 발행하는 경우 충돌을 막습니다."""
        try:
            published = rosgraph.Master(rospy.get_name()).getSystemState()[0]
            publishers = next((nodes for topic, nodes in published
                               if topic == "/cmd_vel"), [])
            return [node for node in publishers if node != rospy.get_name()]
        except Exception:
            return ["ROS_MASTER_UNAVAILABLE"]  # fail closed

    def on_peer_check(self, event):
        sources = self.competing_publishers()  # ROS Master query at 1Hz
        with self.lock:
            self.competing_sources = sources

    def stop(self):
        if hasattr(self, "pub"):
            self.pub.publish(Twist())

    def disable(self, reason):
        """주행을 중지하고 적분항 및 명령 이력까지 모든 제어 상태를 초기화합니다."""
        self.enabled = False
        self.core.reset()
        self.last_tick = None
        self.last_command_v = 0.0
        self.last_command_w = 0.0
        self.stop()
        self.publish_status(reason)

    def on_start(self, req):
        with self.lock:
            if not req.data:
                self.finishing = False
                self.stopped_reports = 0
                self.resume_after_obstacle = False
                self.obstacle_stopped = False
                self.disable("PAUSED")
                return SetBoolResponse(True, "FF-PD PAUSED")
            if self.finishing:
                return SetBoolResponse(False, "GOAL_FINISHING")
            if self.external_hold:
                return SetBoolResponse(False, "EXTERNAL_STOP_REQUEST active")
            if self.motion_policy == "STOP":
                return SetBoolResponse(False, "OBSTACLE_STOP active")
            others = self.competing_publishers()
            if others:
                return SetBoolResponse(False, "REFUSED competing publishers: " + ", ".join(others))
            reason = self.readiness(rospy.Time.now().to_sec())
            if reason:
                return SetBoolResponse(False, "REFUSED: " + reason)
            self.core.reset()
            self.last_tick = None
            self.last_command_v = 0.0
            self.last_command_w = 0.0
            self.resume_after_obstacle = False
            self.obstacle_stopped = False
            self.enabled = True
            self.publish_status("RUNNING")
            return SetBoolResponse(True, "FF-PD STARTED")

    def on_tick(self, event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            others = self.competing_sources
            if others:
                if self.enabled or self.finishing:
                    self.enabled = False
                    self.finishing = False
                    self.core.reset()  # clear I state without racing another publisher
                    self.last_tick = None
                    self.last_command_v = 0.0
                    self.last_command_w = 0.0
                    self.publish_status("COMPETING_CMD_VEL_PUBLISHER")
                    rospy.logerr("FF-PD command conflict: %s", others)
                # 다른 발행자가 있으면 0속도 메시지조차 경쟁 발행하지 않습니다.
                return
            if self.finishing:
                # NUC는 바퀴 상태를 보며 서서히 정지하므로 0속도를 반복 발행합니다.
                # 바퀴 정지가 확인되기 전에는 노드를 종료하지 않습니다.
                self.stop()
                self.publish_status("GOAL_STOPPING")
                if (self.stopped_reports >= STOPPED_REPORTS_TO_FINISH and self.wheel_received is not None
                        and now - self.wheel_received <= self.max_wheel_age):
                    self.publish_status("GOAL_REACHED")
                    rospy.loginfo("FF-PD goal reached; wheels stopped; exiting")
                    rospy.signal_shutdown("GOAL_REACHED")
                return
            if not self.enabled:
                if (self.obstacle_stopped and self.resume_after_obstacle and
                    self.motion_policy in ("CLEAR", "SLOW") and not self.external_hold):
                    if self.readiness(now) is None:
                        self.core.reset()
                        self.enabled = True
                        self.obstacle_stopped = False
                        self.resume_after_obstacle = False
                        self.publish_status("RUNNING")
                if not self.enabled:
                    self.stop()
                    return
            reason = self.readiness(now)
            if reason is None and self.external_hold:
                reason = "EXTERNAL_STOP_REQUEST"
            if reason is not None:
                self.disable("SAFETY_STOP: " + reason)
                return
            dt = 0.1 if self.last_tick is None else now - self.last_tick
            self.last_tick = now
            try:
                pose = self.current_pose(now)
                if pose is None:
                    self.disable("SAFETY_STOP: POSE_STALE")
                    return
                result = self.core.step(
                    pose, self.wheel, now, dt,
                    observation_id=self.pose_seq,
                    observation_time=self.gicp_stamp,
                    motion_policy=self.motion_policy,
                    pitch_deg=self.pitch_deg)
                px, py, _ = result["predicted_pose"]
                result["predicted_in_mask"] = bool(self.mask.contains((px, py)))
                if (result["station"] >= self.route.length - self.goal_tolerance and
                    math.hypot(pose[0] - self.route.xy[-1, 0],
                               pose[1] - self.route.xy[-1, 1]) < self.goal_tolerance):
                    self.disable("GOAL_STOPPING")
                    self.finishing = True
                    self.stopped_reports = 0
                    return
                # Only normal RUNNING commands are limited. disable() sends
                # zero immediately for NO_TRACKING and all other safety faults.
                command_v, command_w = core_settings.limit_running_command(
                    self.last_command_v, self.last_command_w,
                    result["v"], result["w"], dt,
                    self.core.wheel_track, self.core.min_wheel, self.core.max_wheel)
                command = Twist()
                command.linear.x = command_v
                command.angular.z = command_w
                self.pub.publish(command)
                self.last_command_v = command_v
                self.last_command_w = command_w
                self.core.predictor.record(now, command_v, command_w)
                result["command_v"] = command_v
                result["command_w"] = command_w
                self.publish_status("RUNNING", result)
            except Exception as exc:
                self.disable("ERROR_STOP: " + str(exc))
                rospy.logerr("FF-PD computation failed: %s", exc)

    def publish_status(self, state, result=None):
        if state != self.reason or result is not None:
            self.reason = state
            values = {"state": state}
            if result is not None:
                values.update(result)  # publish same FF-PD prediction to Safety
            self.status.publish(String(data=json.dumps(values)))


if __name__ == "__main__":
    try:
        FFNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
