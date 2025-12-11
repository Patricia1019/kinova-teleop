#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse

import rosbag
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
import pdb
# Topic names
RGB_TOPIC      = "/camera/color/image_raw"
POSE_TOPIC     = "/kinova/pose_tool_in_base"
JOINT_TOPIC    = "/kinova/current_joint_state"       # std_msgs/Float64MultiArray
GRIPPER_TOPIC  = "/siemens_demo/gripper_cmd"
CARTVEL_TOPIC  = "/kinova_demo/cart_vel_cmd"         # may or may not be in this bag


def print_topic_summary(bag):
    info = bag.get_type_and_topic_info()
    print("\n=== Topic summary ===")
    if not info.topics:
        print("  (bag contains no topics / messages)")
    else:
        for topic, tinfo in info.topics.items():
            print(f"- {topic}")
            print(f"    type   : {tinfo.msg_type}")
            print(f"    count  : {tinfo.message_count}")
    print("======================\n")


def main():
    parser = argparse.ArgumentParser(
        description="Inspect teleop trajectories stored in a rosbag."
    )
    parser.add_argument("bag_path", help="Path to the .bag file")
    parser.add_argument("--max-steps", type=int, default=10,
                        help="Maximum number of timesteps to print (default: 10)")
    args = parser.parse_args()

    print(f"Opening bag: {args.bag_path}")
    bag = rosbag.Bag(args.bag_path, mode='r')

    # 1) Topic summary
    print_topic_summary(bag)

    info = bag.get_type_and_topic_info()
    if not info.topics:
        bag.close()
        print("Bag is empty, nothing to inspect.")
        return

    # Decide which topic to treat as the "timestep" anchor
    if JOINT_TOPIC in info.topics:
        anchor_topic = JOINT_TOPIC
        anchor_name  = "current_joint_state (Float64MultiArray)"
    elif CARTVEL_TOPIC in info.topics:
        anchor_topic = CARTVEL_TOPIC
        anchor_name  = "cartesian velocity commands"
    else:
        bag.close()
        print("No suitable anchor topic found "
              f"({JOINT_TOPIC} or {CARTVEL_TOPIC} missing).")
        return

    print(f"Anchoring timesteps on: {anchor_topic} ({anchor_name})\n")

    # Only read topics that actually exist in this bag
    topics_to_read = []
    for t in [RGB_TOPIC, POSE_TOPIC, JOINT_TOPIC, GRIPPER_TOPIC, CARTVEL_TOPIC]:
        if t in info.topics:
            topics_to_read.append(t)

    latest_rgb     = None
    latest_pose    = None
    latest_gripper = None

    step_idx = 0
    print(f"Printing up to {args.max_steps} timesteps:\n")

    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        if topic == RGB_TOPIC:
            latest_rgb = msg  # sensor_msgs/Image

        elif topic == POSE_TOPIC:
            latest_pose = msg  # geometry_msgs/PoseStamped

        elif topic == GRIPPER_TOPIC:
            latest_gripper = msg  # std_msgs/Float64MultiArray

        # --- Joint-anchored timesteps (this is your current bag) ---
        elif topic == JOINT_TOPIC and anchor_topic == JOINT_TOPIC:
            step_idx += 1
            if step_idx > args.max_steps:
                break

            js = msg  # std_msgs/Float64MultiArray
            t_sec = t.to_sec()
            print(f"--- Step {step_idx} ---  t = {t_sec:.6f} s")
            print("  Anchor: /kinova/current_joint_state (Float64MultiArray)")

            data = list(js.data)
            print(f"  joint_state.data (len={len(data)}):")
            print(f"    {data}")

            # EE pose
            # pdb.set_trace()
            if latest_pose is not None:
                p = latest_pose.pose.position
                q = latest_pose.pose.orientation
                print("  EE pose (/kinova/pose_tool_in_base):")
                print(f"    position    = [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]")
                print(
                    "    orientation = "
                    f"[x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}]"
                )
            else:
                print("  EE pose: <no pose message received yet>")

            # RGB
            if latest_rgb is not None:
                print("  RGB (/camera/color/image_raw):")
                print(
                    f"    size      = {latest_rgb.width}x{latest_rgb.height}, "
                    f"encoding={latest_rgb.encoding}"
                )
                print(
                    f"    frame_id  = {latest_rgb.header.frame_id}, "
                    f"stamp={latest_rgb.header.stamp.to_sec():.6f}"
                )
            else:
                print("  RGB: <no image received yet>")

            # Gripper
            if latest_gripper is not None:
                print("  Gripper cmd (/siemens_demo/gripper_cmd):")
                print(f"    data = {list(latest_gripper.data)}")
            else:
                print("  Gripper cmd: <no command received yet>")

            print("")

        # --- Optional: cart-vel anchored timesteps (if you ever record that) ---
        elif topic == CARTVEL_TOPIC and anchor_topic == CARTVEL_TOPIC:
            step_idx += 1
            if step_idx > args.max_steps:
                break

            cv = msg  # Float64MultiArray
            t_sec = t.to_sec()
            print(f"--- Step {step_idx} ---  t = {t_sec:.6f} s")
            print("  Anchor: /kinova_demo/cart_vel_cmd (Float64MultiArray)")

            print(f"  action (vx, vy, vz, wx, wy, wz): {list(cv.data)}")

            # EE pose
            if latest_pose is not None:
                p = latest_pose.pose.position
                q = latest_pose.pose.orientation
                print("  EE pose (/kinova/pose_tool_in_base):")
                print(f"    position    = [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]")
                print(
                    "    orientation = "
                    f"[x={q.x:.4f}, y={q.y:.4f}, z={q.z:.4f}, w={q.w:.4f}]"
                )
            else:
                print("  EE pose: <no pose message received yet>")

            # RGB
            if latest_rgb is not None:
                print("  RGB (/camera/color/image_raw):")
                print(
                    f"    size      = {latest_rgb.width}x{latest_rgb.height}, "
                    f"encoding={latest_rgb.encoding}"
                )
                print(
                    f"    frame_id  = {latest_rgb.header.frame_id}, "
                    f"stamp={latest_rgb.header.stamp.to_sec():.6f}"
                )
            else:
                print("  RGB: <no image received yet>")

            # Gripper
            if latest_gripper is not None:
                print("  Gripper cmd (/siemens_demo/gripper_cmd):")
                print(f"    data = {list(latest_gripper.data)}")
            else:
                print("  Gripper cmd: <no command received yet>")

            print("")

    bag.close()
    print("Done.")


if __name__ == "__main__":
    main()
