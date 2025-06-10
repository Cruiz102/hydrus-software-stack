#!/usr/bin/env python3
"""Estimate FPS from cv_publishers log messages.

This script subscribes to /rosout and looks for log messages from the
cv_publishers node that contain "Received RGB image message".
It calculates a rolling FPS over a configurable window of messages.
"""
import collections
import rospy
from rosgraph_msgs.msg import Log


class FPSMonitor:
    def __init__(self, window_size: int = 30) -> None:
        self.window = collections.deque(maxlen=window_size)

    def callback(self, msg: Log) -> None:
        if msg.name.endswith("cv_publihser") and "Received RGB image message" in msg.msg:
            now = rospy.Time.now()
            self.window.append(now)
            if len(self.window) >= 2:
                elapsed = (self.window[-1] - self.window[0]).to_sec()
                if elapsed > 0:
                    fps = (len(self.window) - 1) / elapsed
                    rospy.loginfo(f"Approx FPS: {fps:.2f}")


def main() -> None:
    rospy.init_node("cv_fps_from_logs", anonymous=True)
    window_size = rospy.get_param("~window_size", 30)
    monitor = FPSMonitor(window_size)
    rospy.Subscriber("/rosout", Log, monitor.callback)
    rospy.spin()


if __name__ == "__main__":
    main()
