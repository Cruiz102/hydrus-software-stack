#!/usr/bin/env python3
"""ROS node resource monitor.

This script periodically logs CPU and memory usage for all running ROS nodes.
"""
import subprocess
import time
from typing import Dict, Optional

import psutil
import rospy


def get_nodes() -> list:
    try:
        output = subprocess.check_output(["rosnode", "list"], text=True)
        return [n for n in output.splitlines() if n]
    except subprocess.CalledProcessError:
        return []


def get_pid(node: str) -> Optional[int]:
    try:
        output = subprocess.check_output(["rosnode", "info", node], text=True)
    except subprocess.CalledProcessError:
        return None
    for line in output.splitlines():
        line = line.strip()
        if line.startswith("Pid:"):
            try:
                return int(line.split(":", 1)[1].strip())
            except ValueError:
                return None
    return None


def monitor(interval: float = 1.0) -> None:
    rospy.init_node("ros_node_monitor", anonymous=True)
    last_cpu: Dict[int, float] = {}
    rate = rospy.Rate(1 / interval if interval > 0 else 1)
    while not rospy.is_shutdown():
        for node in get_nodes():
            pid = get_pid(node)
            if pid is None:
                continue
            try:
                proc = psutil.Process(pid)
            except psutil.NoSuchProcess:
                continue
            cpu = proc.cpu_percent(interval=None)
            mem = proc.memory_info().rss / (1024 * 1024)
            rospy.loginfo(f"{node}: CPU {cpu:.2f}% MEM {mem:.2f} MB")
        rate.sleep()


if __name__ == "__main__":
    interval = rospy.get_param("~interval", 1.0)
    monitor(interval)
