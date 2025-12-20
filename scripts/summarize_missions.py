#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
掃描指定資料夾中的 kpi_log_*.csv 檔，
為每一個任務計算 summary，輸出 missions_summary.csv。

一個任務 summary 包含：
- mission_id            : 從檔名取得
- duration_s           : 任務時間（最後一筆 t - 第一筆 t）
- path_length_m        : 以 GT 位置累加的飛行路徑長度
- max_pos_error_m      : e_pos 最大值
- mean_pos_error_m     : e_pos 平均值
- final_pos_error_m    : 最後一筆 e_pos
"""

import os
import glob
import csv
import math
import argparse


def process_kpi_file(path):
    """處理單一 kpi_log 檔案，回傳一個 dict summary。"""
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        first_t = None
        last_t = None

        prev_gt = None  # (x, y, z)
        path_len = 0.0

        max_err = 0.0
        sum_err = 0.0
        cnt = 0
        last_err = 0.0

        for row in reader:
            try:
                t = float(row["t"])
                x_gt = float(row["px_gt"])
                y_gt = float(row["py_gt"])
                z_gt = float(row["pz_gt"])
                e_pos = float(row["e_pos"])
            except (KeyError, ValueError):
                # 欄位缺失或轉型失敗，略過這行
                continue

            if first_t is None:
                first_t = t
            last_t = t

            # 累積路徑長度（使用 GT 位置）
            if prev_gt is not None:
                dx = x_gt - prev_gt[0]
                dy = y_gt - prev_gt[1]
                dz = z_gt - prev_gt[2]
                path_len += math.sqrt(dx * dx + dy * dy + dz * dz)
            prev_gt = (x_gt, y_gt, z_gt)

            # 誤差統計
            if e_pos > max_err:
                max_err = e_pos
            sum_err += e_pos
            cnt += 1
            last_err = e_pos

        if cnt == 0 or first_t is None or last_t is None:
            # 沒有有效資料
            return None

        duration = last_t - first_t
        mean_err = sum_err / float(cnt)

        summary = {
            "duration_s": duration,
            "path_length_m": path_len,
            "max_pos_error_m": max_err,
            "mean_pos_error_m": mean_err,
            "final_pos_error_m": last_err,
        }
        return summary


def main():
    parser = argparse.ArgumentParser(
        description="Summarize multiple kpi_log_*.csv files into missions_summary.csv"
    )
    parser.add_argument(
    "--log_dir",
    type=str,
    default="/home/tim/laea/src/laea_twin_tools/laea_logs",
    help="kpi_log_*.csv 存放的資料夾 (default: /home/tim/laea/src/laea_twin_tools/laea_logs)",
)
    parser.add_argument(
        "--output",
        type=str,
        default="missions_summary.csv",
        help="輸出 summary 檔名稱 (default: missions_summary.csv)",
    )
    args = parser.parse_args()

    log_dir = os.path.abspath(os.path.expanduser(args.log_dir))
    if not os.path.isdir(log_dir):
        print("[summarize_missions] log_dir 不存在：", log_dir)
        return

    pattern = os.path.join(log_dir, "kpi_log_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print("[summarize_missions] 找不到任何 kpi_log_*.csv，路徑：", pattern)
        return

    print("[summarize_missions] 發現 %d 個任務 log" % len(files))

    # summary 輸出檔路徑
    summary_path = os.path.join(log_dir, args.output)
    with open(summary_path, "w") as f_out:
        fieldnames = [
            "mission_id",
            "duration_s",
            "path_length_m",
            "max_pos_error_m",
            "mean_pos_error_m",
            "final_pos_error_m",
        ]
        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        for path in files:
            mission_id = os.path.splitext(os.path.basename(path))[0]
            # 例如 kpi_log_1700000000 -> mission_id
            print("[summarize_missions] 處理", mission_id)

            summary = process_kpi_file(path)
            if summary is None:
                print("  -> 無有效資料，略過")
                continue

            row = {"mission_id": mission_id}
            row.update(summary)
            writer.writerow(row)

    print("[summarize_missions] 已產生 summary 檔：", summary_path)


if __name__ == "__main__":
    main()
