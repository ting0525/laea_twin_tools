#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
import math

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped


class SlamKpiLogger(object):
    def __init__(self):
        # =========================
        # 參數：兼容你目前 launch 的 global param 風格
        # =========================
        self.model_name = rospy.get_param(
            "/slam_kpi_logger/model_name",
            rospy.get_param("~model_name", "iris_0")
        )

        self.output_dir = rospy.get_param(
            "/slam_kpi_logger/output_dir",
            "/home/tim/laea/src/laea_twin_tools/laea_logs"
        )

        # output_name 會被 launch 組成：kpi_log_<run_id>，例如 kpi_log_test_001
        raw_output_name = rospy.get_param("/slam_kpi_logger/output_name", "kpi_log")

        # ✅ 修正：若沒給 run_id，launch 會變成 kpi_log_，這裡自動補 timestamp，避免產生 kpi_log_.csv
        if (raw_output_name is None) or (str(raw_output_name).strip() == "") or str(raw_output_name).endswith("_"):
            ts = int(rospy.get_time())
            self.output_name = "kpi_log_%d" % ts
        else:
            self.output_name = str(raw_output_name)

        # log_rate：你 launch 已有 /slam_kpi_logger/log_rate
        self.rate_hz = float(rospy.get_param(
            "/slam_kpi_logger/log_rate",
            rospy.get_param("~log_rate", 20.0)
        ))

        # =========================
        # 建立輸出路徑（固定檔名）
        # =========================
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)

        self.file_path = os.path.join(self.output_dir, "%s.csv" % self.output_name)

        # 公告：供 batch manager 失敗時刪檔
        rospy.set_param("/laea_twin/current_log_path", self.file_path)
        rospy.set_param("/laea_twin/current_output_name", self.output_name)

        rospy.loginfo("[slam_kpi_logger] model_name: %s", self.model_name)
        rospy.loginfo("[slam_kpi_logger] log_rate: %.2f Hz", self.rate_hz)
        rospy.loginfo("[slam_kpi_logger] output_name(raw): %s", str(raw_output_name))
        rospy.loginfo("[slam_kpi_logger] output_name(final): %s", self.output_name)
        rospy.loginfo("[slam_kpi_logger] log file: %s", self.file_path)

        # 開檔 + 寫入 CSV header
        self.csv_file = open(self.file_path, "w")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "t",
            "px_gt", "py_gt", "pz_gt",
            "px_est", "py_est", "pz_est",
            "e_pos"
        ])
        self.csv_file.flush()

        # 最新 ground truth / estimate
        self.latest_gt = None   # (t, x, y, z)
        self.latest_est = None  # (t, x, y, z)

        # =========================
        # Subscribers
        # =========================
        self.sub_states = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.cb_model_states, queue_size=10
        )
        self.sub_pose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.cb_pose, queue_size=50
        )

        # Timer（以 ROS simulated time 為主）
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.cb_timer)

    def cb_model_states(self, msg):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0,
                "[slam_kpi_logger] model '%s' not found in /gazebo/model_states",
                self.model_name
            )
            return

        pose = msg.pose[idx]

        # Gazebo 的 ModelStates 通常沒有 header，統一用 rospy.get_time()
        t = rospy.get_time()
        self.latest_gt = (t, pose.position.x, pose.position.y, pose.position.z)

    def cb_pose(self, msg):
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        p = msg.pose.position
        self.latest_est = (t, p.x, p.y, p.z)

    def cb_timer(self, event):
        if self.latest_gt is None or self.latest_est is None:
            return

        # 用 simulated time（若 /use_sim_time=true）
        t = rospy.get_time()

        _, x_gt, y_gt, z_gt = self.latest_gt
        _, x_est, y_est, z_est = self.latest_est

        e_pos = math.sqrt(
            (x_gt - x_est) ** 2 +
            (y_gt - y_est) ** 2 +
            (z_gt - z_est) ** 2
        )

        self.writer.writerow([
            t,
            x_gt, y_gt, z_gt,
            x_est, y_est, z_est,
            e_pos
        ])

        # 每筆 flush：確保中途 kill logger 也不太會丟資料
        self.csv_file.flush()

    def shutdown(self):
        rospy.loginfo("[slam_kpi_logger] shutting down")
        try:
            self.timer.shutdown()
        except Exception:
            pass
        try:
            self.csv_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("slam_kpi_logger")
    node = SlamKpiLogger()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
