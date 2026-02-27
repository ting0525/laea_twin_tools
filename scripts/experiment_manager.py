#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import signal
import subprocess
import rospy

from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Log as RosLog


class ExperimentManager:
    def __init__(self):
        rospy.init_node("experiment_manager", anonymous=False)

        # ----------------------------
        # Orchestration params
        # ----------------------------
        self.num_runs = int(rospy.get_param("~num_runs", 1))
        self.sleep_between_runs_s = float(rospy.get_param("~sleep_between_runs_s", 2.0))

        # Trigger
        self.start_topic = rospy.get_param("~start_topic", "/traj_start_trigger")
        self.start_frame_id = rospy.get_param("~start_frame_id", "map")
        self.start_pub = rospy.Publisher(self.start_topic, PoseStamped, queue_size=1, latch=True)

        # ----------------------------
        # Success token (唯一成功標準)
        # ----------------------------
        self.rosout_topic = rospy.get_param("~rosout_topic", "/rosout_agg")
        self.finish_token = rospy.get_param("~finish_token", "finish exploration.")
        # 強烈建議後續填上實際 node 名稱（例如 /fast_exploration_fsm），避免誤判
        self.finish_node_name = rospy.get_param("~finish_node_name", "")  # "" = 不限制
        rospy.Subscriber(self.rosout_topic, RosLog, self._rosout_cb, queue_size=200)

        # ----------------------------
        # Fail detection (GT vs EST)
        # ----------------------------
        self.model_name = rospy.get_param("~model_name", "iris_0")
        self.fail_error_m = float(rospy.get_param("~fail_error_m", 10.0))
        self.fail_hold_s = float(rospy.get_param("~fail_hold_s", 1.0))

        # Timeout: 沒 finish 一律視為非成功（刪檔）
        self.max_duration_s = float(rospy.get_param("~max_duration_s", 900.0))

        # ----------------------------
        # Logger control
        # ----------------------------
        self.output_dir = rospy.get_param(
            "~output_dir",
            rospy.get_param(
                "/slam_kpi_logger/output_dir",
                os.path.expanduser("~/laea/src/laea_twin_tools/laea_logs")
            )
        )
        # 這個參數你可以不改，維持 roslaunch <pkg> <launch>
        self.logger_launch = rospy.get_param("~logger_launch", "laea_twin_tools slam_kpi_logger.launch")

        # Dataset governance
        self.delete_on_non_success = bool(rospy.get_param("~delete_on_non_success", True))

        # ----------------------------
        # Persistent Run ID (解決覆蓋的關鍵)
        # ----------------------------
        # 序號檔：跨重啟唯一遞增，確保永不覆蓋
        self.run_seq_file = rospy.get_param("~run_seq_file", os.path.join(self.output_dir, "run_seq.txt"))

        # ----------------------------
        # Data inputs
        # ----------------------------
        self.gt_xyz = None
        self.est_xyz = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._gt_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._est_cb, queue_size=50)

        # Internal state
        self._run_start_time = None
        self._finish_seen = False
        self._finish_time = None
        self._fail_start_time = None

        self._logger_proc = None

    # ----------------------------
    # Callbacks
    # ----------------------------
    def _rosout_cb(self, msg: RosLog):
        if self._run_start_time is None:
            return
        if msg.header.stamp < self._run_start_time:
            return
        if self.finish_node_name and (msg.name != self.finish_node_name):
            return
        if self.finish_token in (msg.msg or ""):
            self._finish_seen = True
            self._finish_time = msg.header.stamp

    def _gt_cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        p = msg.pose[idx].position
        self.gt_xyz = (p.x, p.y, p.z)

    def _est_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.est_xyz = (p.x, p.y, p.z)

    # ----------------------------
    # Helpers
    # ----------------------------
    def _compute_e_pos(self):
        if self.gt_xyz is None or self.est_xyz is None:
            return None
        dx = self.gt_xyz[0] - self.est_xyz[0]
        dy = self.gt_xyz[1] - self.est_xyz[1]
        dz = self.gt_xyz[2] - self.est_xyz[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wait_ready(self, timeout_s=60.0):
        """最小 readiness gate：確保 GT/EST 都有資料"""
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.gt_xyz is not None and self.est_xyz is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout_s:
                return False
            rate.sleep()
        return False

    def _publish_start_trigger(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.start_frame_id
        # 位置內容依你 waypoint_generator 行為；大多數情況 frame_id 正確即足夠
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.start_pub.publish(msg)

    def _next_global_run_id(self) -> str:
        """跨重啟的全域 run 序號：避免永遠 run_001 覆蓋"""
        os.makedirs(self.output_dir, exist_ok=True)

        n = 0
        if os.path.isfile(self.run_seq_file):
            try:
                with open(self.run_seq_file, "r") as f:
                    n = int((f.read() or "0").strip())
            except Exception:
                n = 0

        n += 1

        # 原子更新：避免寫到一半中斷造成 seq 壞掉
        tmp_path = self.run_seq_file + ".tmp"
        with open(tmp_path, "w") as f:
            f.write(str(n))
        os.replace(tmp_path, self.run_seq_file)

        return f"run_{n:03d}"

    def _start_logger(self, run_id: str):
        """
        每個 run 啟動一次 logger，並把 run_id 帶進去
        讓 logger 輸出：kpi_log_<run_id>.csv
        """
        if self._logger_proc is not None and self._logger_proc.poll() is None:
            rospy.logwarn("Logger already running, terminating previous instance.")
            self._stop_logger()

        cmd = ["roslaunch"] + self.logger_launch.split() + [
            f"run_id:={run_id}",
            f"output_dir:={self.output_dir}",
        ]
        rospy.loginfo(f"[RUN {run_id}] start logger: {' '.join(cmd)}")

        self._logger_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        # 給 logger 時間 set /laea_twin/current_log_path
        rospy.sleep(1.0)

        # 可選：快速稽核 current_log_path
        try:
            path = rospy.get_param("/laea_twin/current_log_path", "")
            if not path:
                rospy.logwarn(f"[RUN {run_id}] current_log_path is empty after logger start.")
            else:
                rospy.loginfo(f"[RUN {run_id}] current_log_path={path}")
        except Exception:
            rospy.logwarn(f"[RUN {run_id}] unable to read /laea_twin/current_log_path after logger start.")

    def _stop_logger(self):
        if self._logger_proc is None:
            return
        if self._logger_proc.poll() is not None:
            self._logger_proc = None
            return
        try:
            os.killpg(os.getpgid(self._logger_proc.pid), signal.SIGINT)
        except Exception:
            pass
        rospy.sleep(0.5)
        self._logger_proc = None

    def _delete_current_log(self, reason: str):
        if not self.delete_on_non_success:
            return

        try:
            path = rospy.get_param("/laea_twin/current_log_path", "")
        except KeyError:
            path = ""

        if not path:
            rospy.logwarn(f"[DELETE] ({reason}) current_log_path param not set; skip delete.")
            return

        if os.path.isfile(path):
            try:
                os.remove(path)
                rospy.logwarn(f"[DELETE] ({reason}) removed log: {path}")
            except Exception as e:
                rospy.logerr(f"[DELETE] ({reason}) failed to remove {path}: {e}")
        else:
            rospy.logwarn(f"[DELETE] ({reason}) file not found: {path}")

    # ----------------------------
    # Run lifecycle
    # ----------------------------
    def _reset_run_flags(self):
        self._run_start_time = rospy.Time.now()
        self._finish_seen = False
        self._finish_time = None
        self._fail_start_time = None

    def _monitor_one_run(self, run_id: str):
        """
        Return outcome:
          - SUCCESS_FINISH
          - FAIL_SLAM
          - TIMEOUT_NO_FINISH
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # 1) success: finish token (唯一成功標準)
            if self._finish_seen:
                rospy.loginfo(f"[RUN {run_id}] SUCCESS_FINISH at {self._finish_time.to_sec():.3f}")
                return "SUCCESS_FINISH"

            # 2) fail: e_pos hold
            epos = self._compute_e_pos()
            if epos is not None and epos > self.fail_error_m:
                if self._fail_start_time is None:
                    self._fail_start_time = rospy.Time.now()
                else:
                    held = (rospy.Time.now() - self._fail_start_time).to_sec()
                    if held >= self.fail_hold_s:
                        rospy.logerr(f"[RUN {run_id}] FAIL_SLAM e_pos={epos:.2f}m held={held:.2f}s")
                        return "FAIL_SLAM"
            else:
                self._fail_start_time = None

            # 3) timeout: no finish
            elapsed = (rospy.Time.now() - self._run_start_time).to_sec()
            if elapsed >= self.max_duration_s:
                rospy.logwarn(f"[RUN {run_id}] TIMEOUT_NO_FINISH elapsed={elapsed:.1f}s")
                return "TIMEOUT_NO_FINISH"

            rate.sleep()

        return "ABORTED"

    def run(self):
        rospy.loginfo(f"[experiment_manager] num_runs={self.num_runs}, output_dir={self.output_dir}")
        ok = self._wait_ready(timeout_s=60.0)
        if not ok:
            rospy.logerr("[experiment_manager] Inputs not ready: missing GT/EST. Abort.")
            return

        for _ in range(self.num_runs):
            run_id = self._next_global_run_id()
            rospy.loginfo(f"========== RUN {run_id} ==========")

            self._reset_run_flags()

            # 先開 logger，再 trigger（避免漏掉起始資料）
            self._start_logger(run_id)
            self._publish_start_trigger()

            outcome = self._monitor_one_run(run_id)

            # stop logger
            self._stop_logger()

            # governance: only keep SUCCESS_FINISH
            if outcome != "SUCCESS_FINISH":
                self._delete_current_log(reason=outcome)
            else:
                rospy.loginfo(f"[RUN {run_id}] kept log (SUCCESS_FINISH).")

            rospy.sleep(self.sleep_between_runs_s)

        rospy.loginfo("[experiment_manager] All runs completed.")


if __name__ == "__main__":
    try:
        mgr = ExperimentManager()
        mgr.run()
    except rospy.ROSInterruptException:
        pass
