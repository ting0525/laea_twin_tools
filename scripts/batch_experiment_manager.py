#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import rospy
import subprocess
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates


class BatchExperimentManager(object):
    """
    DT1 ?????? Orchestrator
    - ??? Gazebo/PX4/Exploration/RViz?????????
    - ??? N ? (trigger + logger)
    - SLAM fail ? ????????? log?????
    """

    def __init__(self):
        # ----- batch control -----
        self.num_runs = int(rospy.get_param("~num_runs", 20))
        self.sleep_between_runs_s = float(rospy.get_param("~sleep_between_runs_s", 3.0))

        # ----- trigger -----
        self.start_topic = rospy.get_param("~start_topic", "/traj_start_trigger")
        self.start_frame_id = rospy.get_param("~start_frame_id", "map")  # ???? pose frame_id
        self.start_pub_repeat = int(rospy.get_param("~start_pub_repeat", 5))

        # ----- readiness -----
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.pose_topic = rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        self.ready_timeout_s = float(rospy.get_param("~ready_timeout_s", 60.0))

        # ----- mission end conditions -----
        self.max_duration_s = float(rospy.get_param("~max_duration_s", 900.0))
        self.stationary_window_s = float(rospy.get_param("~stationary_window_s", 60.0))
        self.stationary_eps_m = float(rospy.get_param("~stationary_eps_m", 0.25))

        # ----- SLAM fail detection (GT vs Est) -----
        self.gt_topic = rospy.get_param("~gt_topic", "/gazebo/model_states")
        self.gt_model_name = rospy.get_param("~gt_model_name", "iris_0")
        self.fail_error_m = float(rospy.get_param("~fail_error_m", 10.0))
        self.fail_hold_s = float(rospy.get_param("~fail_hold_s", 1.0))

        # ----- logger control -----
        self.logger_launch_pkg = rospy.get_param("~logger_launch_pkg", "laea_twin_tools")
        self.logger_launch_file = rospy.get_param("~logger_launch_file", "slam_kpi_logger.launch")

        self.log_dir = rospy.get_param("~log_dir", "/home/tim/laea/src/laea_twin_tools/laea_logs")
        self.current_log_param = rospy.get_param("~current_log_param", "/laea_twin/current_log_path")

        # ----- runtime states -----
        self.mav_state = None
        self.last_est_pose = None  # (t, x, y, z)
        self.last_gt_pose = None   # (t, x, y, z)
        self.pose_history = []
        self.fail_start_time = None

        # ----- ROS I/O -----
        rospy.Subscriber(self.state_topic, State, self.cb_state, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_est_pose, queue_size=50)
        rospy.Subscriber(self.gt_topic, ModelStates, self.cb_gt, queue_size=10)

        self.start_pub = rospy.Publisher(self.start_topic, PoseStamped, queue_size=1, latch=True)

        if not os.path.isdir(self.log_dir):
            os.makedirs(self.log_dir)

        rospy.loginfo("[batch_manager] init ok. num_runs=%d, log_dir=%s", self.num_runs, self.log_dir)

    def cb_state(self, msg):
        self.mav_state = msg

    def cb_est_pose(self, msg):
        p = msg.pose.position
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        self.last_est_pose = (t, p.x, p.y, p.z)

        # stationary history
        self.pose_history.append(self.last_est_pose)
        cutoff = t - self.stationary_window_s
        while self.pose_history and self.pose_history[0][0] < cutoff:
            self.pose_history.pop(0)

    def cb_gt(self, msg):
        try:
            idx = msg.name.index(self.gt_model_name)
        except ValueError:
            return
        p = msg.pose[idx].position
        t = rospy.get_time()
        self.last_gt_pose = (t, p.x, p.y, p.z)

    def wait_until_ready(self):
        t0 = rospy.get_time()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            mode_ok = (self.mav_state is not None and self.mav_state.mode == "OFFBOARD")
            armed_ok = (self.mav_state is not None and bool(self.mav_state.armed))
            pose_ok = (self.last_est_pose is not None)

            if mode_ok and armed_ok and pose_ok:
                rospy.loginfo("[batch_manager] READY: OFFBOARD+ARMED+POSE")
                return True

            if rospy.get_time() - t0 > self.ready_timeout_s:
                rospy.logwarn("[batch_manager] READY timeout. Continue anyway.")
                return False

            rate.sleep()

    def publish_start_trigger(self):
        if self.last_est_pose is None:
            rospy.logwarn("[batch_manager] cannot trigger: no est pose")
            return False

        _, x, y, z = self.last_est_pose

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.start_frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0

        rospy.loginfo("[batch_manager] publish trigger to %s frame=%s", self.start_topic, self.start_frame_id)
        for _ in range(self.start_pub_repeat):
            self.start_pub.publish(msg)
            rospy.sleep(0.2)
        return True

    def moved_distance_in_window(self):
        if len(self.pose_history) < 2:
            return None
        _, x0, y0, z0 = self.pose_history[0]
        _, x1, y1, z1 = self.pose_history[-1]
        dx, dy, dz = x1-x0, y1-y0, z1-z0
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def compute_e_pos(self):
        if self.last_est_pose is None or self.last_gt_pose is None:
            return None
        _, ex, ey, ez = self.last_est_pose
        _, gx, gy, gz = self.last_gt_pose
        dx, dy, dz = ex-gx, ey-gy, ez-gz
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def start_logger(self, run_id):
        """
        ? roslaunch ?? logger???? run_id/output_dir ??
        """
        cmd = [
            "roslaunch",
            "laea_twin_tools",
            "slam_kpi_logger.launch",
            "run_id:=%s" % str(run_id),
            "output_dir:=%s" % self.log_dir,
        ]

        rospy.loginfo("[batch_manager] start logger: %s", " ".join(cmd))
        return subprocess.Popen(cmd)

    def stop_logger(self, proc):
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    def delete_failed_log(self):
        """
        ?? logger ????? log
        ??? slam_kpi_logger.py ?? /laea_twin/current_log_path?
        """
        if rospy.has_param(self.current_log_param):
            path = rospy.get_param(self.current_log_param)
            if path and os.path.isfile(path):
                rospy.logwarn("[batch_manager] delete failed log: %s", path)
                try:
                    os.remove(path)
                    return True
                except Exception as e:
                    rospy.logwarn("[batch_manager] delete failed: %s", str(e))
        return False

    def run_one(self, idx):
        self.fail_start_time = None
        self.pose_history = []

        # 1) ?? ready
        self.wait_until_ready()

        # 2) ?? logger
        run_id = "run_%03d" % idx
        logger_proc = self.start_logger(run_id)
        rospy.sleep(1.0)

        # 3) trigger
        self.publish_start_trigger()

        # 4) monitor
        t0 = rospy.get_time()
        rate = rospy.Rate(10)
        status = "SUCCESS"

        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - t0

            # ???????
            if elapsed > self.max_duration_s:
                status = "SUCCESS_TIMEOUT"
                break

            # ???????
            dist = self.moved_distance_in_window()
            if dist is not None and dist < self.stationary_eps_m:
                status = "SUCCESS_STATIONARY"
                break

            # ???SLAM e_pos ?????
            epos = self.compute_e_pos()
            if epos is not None and epos > self.fail_error_m:
                if self.fail_start_time is None:
                    self.fail_start_time = rospy.get_time()
                elif rospy.get_time() - self.fail_start_time >= self.fail_hold_s:
                    status = "FAIL_SLAM"
                    break
            else:
                self.fail_start_time = None

            rate.sleep()

        # 5) stop logger
        self.stop_logger(logger_proc)
        rospy.sleep(0.5)

        # 6) fail -> delete log
        if status == "FAIL_SLAM":
            self.delete_failed_log()

        rospy.loginfo("[batch_manager] %s finished: %s", run_id, status)
        return status

    def run(self):
        summary = {}
        for i in range(1, self.num_runs + 1):
            rospy.loginfo("========== RUN %d / %d ==========", i, self.num_runs)
            status = self.run_one(i)
            summary[status] = summary.get(status, 0) + 1
            rospy.sleep(self.sleep_between_runs_s)

        rospy.loginfo("[batch_manager] DONE summary=%s", str(summary))


if __name__ == "__main__":
    rospy.init_node("batch_experiment_manager", anonymous=False)
    BatchExperimentManager().run()
