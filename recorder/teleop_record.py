#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
import rosbag

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped


# Globals for latest messages
latest_action = None      # /kinova_demo/cart_vel_cmd
latest_rgb    = None      # /camera/color/image_raw
latest_pose   = None      # /kinova/pose_tool_in_base

lock = threading.Lock()


def action_cb(msg):
    global latest_action
    with lock:
        latest_action = msg


def rgb_cb(msg):
    global latest_rgb
    with lock:
        latest_rgb = msg


def pose_cb(msg):
    global latest_pose
    with lock:
        latest_pose = msg


def main():
    rospy.init_node("teleop_trajectory_recorder")

    # Bag path (can override with _bag_path:=... in rosrun/roslaunch)
    bag_path = rospy.get_param("~bag_path", "./rosbag/teleop_trajectory.bag")
    log_hz   = rospy.get_param("~log_hz", 50.0)  # match your teleop LOOP_HZ

    rospy.loginfo("Trajectory recorder starting.")
    rospy.loginfo("  Bag path : %s", bag_path)
    rospy.loginfo("  Log rate : %.1f Hz", log_hz)

    # Open bag for writing
    bag = rosbag.Bag(bag_path, mode='w')

    def shutdown_hook():
        rospy.loginfo("Shutting down, closing bag...")
        bag.close()
        rospy.loginfo("Bag closed.")

    rospy.on_shutdown(shutdown_hook)

    # Subscribers
    rospy.Subscriber("/kinova_demo/cart_vel_cmd",
                     Float64MultiArray, action_4cb, queue_size=10)
    rospy.Subscriber("/camera/color/image_raw",
                     Image, rgb_cb, queue_size=10)
    rospy.Subscriber("/kinova/pose_tool_in_base",
                     PoseStamped, pose_cb, queue_size=10)

    rate = rospy.Rate(log_hz)

    try:
        while not rospy.is_shutdown():
            with lock:
                a = latest_action
                rgb = latest_rgb
                pose = latest_pose

            # Only log when we have all three
            if a is not None and rgb is not None and pose is not None:
                # Use a common timestamp for this "timestep"
                t = rospy.Time.now()

                # These writes are exactly what rosbag record would do,
                # but now grouped at the same time t.
                bag.write("/kinova_demo/cart_vel_cmd", a, t)
                bag.write("/camera/color/image_raw",   rgb, t)
                bag.write("/kinova/pose_tool_in_base", pose, t)

            rate.sleep()

    # In case shutdown_hook didn’t run for some reason:
    finally:
        try:
            bag.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
