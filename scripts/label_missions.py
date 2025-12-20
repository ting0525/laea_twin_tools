#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
?? missions_summary.csv?????????????? success ???
?? missions_labeled.csv?

???????
- max_pos_error_m   < 2.0
- mean_pos_error_m  < 1.5
- final_pos_error_m < 2.0
"""

import os
import csv
import argparse


def label_one_row(row):
    """?????? success=1/0?????????? None?"""
    try:
        max_err = float(row["max_pos_error_m"])
        mean_err = float(row["mean_pos_error_m"])
        final_err = float(row["final_pos_error_m"])
    except (KeyError, ValueError):
        return None

    if (max_err < 2.0) and (mean_err < 1.5) and (final_err < 2.0):
        return 1
    else:
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Label missions as success/fail based on summary metrics."
    )
    parser.add_argument(
        "--log_dir",
        type=str,
        default="/home/tim/laea/src/laea_twin_tools/laea_logs",
        help="missions_summary.csv ?????? (default: /home/tim/laea/src/laea_twin_tools/laea_logs)",
    )
    parser.add_argument(
        "--input",
        type=str,
        default="missions_summary.csv",
        help="?? summary ???? (default: missions_summary.csv)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="missions_labeled.csv",
        help="output lable file name (default: missions_labeled.csv)",
    )
    args = parser.parse_args()

    log_dir = os.path.abspath(os.path.expanduser(args.log_dir))
    summary_path = os.path.join(log_dir, args.input)

    if not os.path.isfile(summary_path):
        print("[label_missions] not find file", summary_path)
        return

    output_path = os.path.join(log_dir, args.output)

    print("[label_missions] ???", summary_path)
    print("[label_missions] ???", output_path)

    with open(summary_path, "r") as f_in, open(output_path, "w") as f_out:
        reader = csv.DictReader(f_in)
        fieldnames = reader.fieldnames[:] if reader.fieldnames else []

        # ???? success ???????????
        if "success" not in fieldnames:
            fieldnames.append("success")

        writer = csv.DictWriter(f_out, fieldnames=fieldnames)
        writer.writeheader()

        count_total = 0
        count_ok = 0

        for row in reader:
            count_total += 1
            label = label_one_row(row)
            if label is None:
                print("[label_missions] number %d error" % count_total)
                continue
            row["success"] = label
            writer.writerow(row)
            count_ok += 1

    print("[label_missions] 總任務數量: %d, 成功標註數%d" % (count_total, count_ok))


if __name__ == "__main__":
    main()
