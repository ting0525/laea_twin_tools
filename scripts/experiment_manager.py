#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import subprocess

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


class ExperimentManager(object):
    """
    DT1 實驗協調節點：
    1) 等待 OFFBOARD + ARMED（可選 timeout）
    2) 自動 publish /traj_start_trigger（取代 RViz 2D Nav Goal 的啟動效果）
    3) 監控 UAV 是否長時間不動，或超時
    4) 任務結束後自動 kill KPI logger node
    """

    def __init__(self):
        # ---------- Topics ----------
        self.pose_topic = rospy.get_param("~pose_topic", "/mavros/local_position/pose")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")

        # 探索啟動 Trigger（你系統的正確入口）
        self.start_topic = rospy.get_param("~start_topic", "/traj_start_trigger")

        # ⚠️ frame_id 請優先設定成你 pose 的 frame_id（可用下面的驗證指令取得）
        self.start_frame_id = rospy.get_param("~start_frame_id", "map")

        # 發送 trigger 次數（避免 subscriber 啟動過程漏接）
        self.start_pub_repeat = int(rospy.get_param("~start_pub_repeat", 5))

        # ---------- Mission End Conditions ----------
        # 任務最長時間（秒）
        self.max_duration_s = float(rospy.get_param("~max_duration_s", 900.0))  # 15 min

        # stationary 判斷：window 內移動距離 < eps → 視為任務完成/卡住
        self.stationary_window_s = float(rospy.get_param("~stationary_window_s", 60.0))
        self.stationary_eps_m = float(rospy.get_param("~stationary_eps_m", 0.25))

        # ---------- Readiness ----------
        # 等待 OFFBOARD + ARMED 最長時間
        self.ready_timeout_s = float(rospy.get_param("~ready_timeout_s", 60.0))

        # ---------- Nodes to Stop ----------
        # 你的 KPI logger node 名稱（若不同請改參數）
        self.logger_node = rospy.get_param("~logger_node", "/slam_kpi_logger")

        # ---------- Runtime State ----------
        self.mav_state = None
        self.last_pose = None  # (t, x, y, z)
        self.pose_history = []  # 最近 stationary_window_s 的 pose (t,x,y,z)
        self.start_time = None

        # ---------- ROS I/O ----------
        rospy.Subscriber(self.state_topic, State, self.cb_state, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=50)

        self.start_pub = rospy.Publisher(self.start_topic, PoseStamped, queue_size=1, latch=True)

        rospy.loginfo("[experiment_manager] started.")
        rospy.loginfo("[experiment_manager] start_topic=%s frame=%s", self.start_topic, self.start_frame_id)

    def cb_state(self, msg):
        self.mav_state = msg

    def cb_pose(self, msg):
        p = msg.pose.position
        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        self.last_pose = (t, p.x, p.y, p.z)

        # 保留最近 stationary_window_s 的歷史
        self.pose_history.append(self.last_pose)
        cutoff = t - self.stationary_window_s
        while self.pose_history and self.pose_history[0][0] < cutoff:
            self.pose_history.pop(0)

    def wait_until_ready(self):
        """
        等待 OFFBOARD + ARMED（避免探索還沒開始就計時）
        若 timeout 仍未達成，仍會繼續往下跑（利於 debug / 不阻塞流程）
        """
        t0 = rospy.get_time()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            mode_ok = False
            armed_ok = False

            if self.mav_state is not None:
                mode_ok = (self.mav_state.mode == "OFFBOARD")
                armed_ok = bool(self.mav_state.armed)

            if mode_ok and armed_ok and self.last_pose is not None:
                rospy.loginfo("[experiment_manager] READY: OFFBOARD + ARMED confirmed.")
                return True

            if rospy.get_time() - t0 > self.ready_timeout_s:
                rospy.logwarn("[experiment_manager] READY timeout. Continue anyway.")
                return False

            rate.sleep()

    def publish_start_trigger(self):
        """
        取代 RViz 2D Nav Goal 的效果：對 /traj_start_trigger 發一個 PoseStamped。
        建議用 UAV 當下位姿作為 trigger 的 position，避免 frame mismatch。
        """
        if self.last_pose is None:
            rospy.logwarn("[experiment_manager] No pose yet, cannot publish start trigger.")
            return False

        _, x, y, z = self.last_pose

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.start_frame_id

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # orientation 不重要，給 unit quaternion
        msg.pose.orientation.w = 1.0

        rospy.loginfo(
            "[experiment_manager] Publishing start trigger (%s) frame=%s at (%.2f, %.2f, %.2f)",
            self.start_topic, self.start_frame_id, x, y, z
        )

        for _ in range(self.start_pub_repeat):
            self.start_pub.publish(msg)
            rospy.sleep(0.3)

        return True

    def moved_distance_in_window(self):
        """
        計算 stationary_window_s 內的移動距離（首尾距離近似）
        """
        if len(self.pose_history) < 2:
            return None
        _, x0, y0, z0 = self.pose_history[0]
        _, x1, y1, z1 = self.pose_history[-1]
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def kill_node(self, node_name):
        try:
            subprocess.check_call(["rosnode", "kill", node_name])
            rospy.loginfo("[experiment_manager] killed node: %s", node_name)
            return True
        except Exception as e:
            rospy.logwarn("[experiment_manager] failed to kill node %s: %s", node_name, str(e))
            return False

    def run(self):
        # 1) 等待 ready（OFFBOARD+ARM）
        self.wait_until_ready()

        # 2) 發探索啟動 trigger（等同 RViz）
        self.publish_start_trigger()

        # 3) 開始計時與監控
        self.start_time = rospy.get_time()
        rospy.loginfo("[experiment_manager] mission started. max_duration_s=%.1f", self.max_duration_s)

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            elapsed = now - self.start_time

            # 條件 A：超時
            if elapsed > self.max_duration_s:
                rospy.loginfo("[experiment_manager] DONE: timeout reached (%.1fs).", elapsed)
                break

            # 條件 B：長時間不動（探索完成/卡住）
            dist = self.moved_distance_in_window()
            if dist is not None and dist < self.stationary_eps_m:
                rospy.loginfo(
                    "[experiment_manager] DONE: stationary detected. dist=%.3fm in %.1fs",
                    dist, self.stationary_window_s
                )
                break

            rate.sleep()

        # 4) 任務結束：停止 logger
        self.kill_node(self.logger_node)
        rospy.loginfo("[experiment_manager] experiment finished.")


if __name__ == "__main__":
    rospy.init_node("experiment_manager", anonymous=False)
    node = ExperimentManager()
    node.run()

