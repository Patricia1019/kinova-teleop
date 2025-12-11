#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import termios
import threading

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from pynput import keyboard
import tf.transformations as tft

LIN_SPEED = 0.15   # m/s   (BASE frame)
ANG_SPEED = 0.6    # rad/s (TOOL frame)
LOOP_HZ   = 50.0   # Hz

# Home position in BASE frame (your updated value)
HOME_POS_BASE = np.array([0.242, 0.018, 0.211], dtype=float)

# Homing controller params (tuned to avoid oscillation)
HOME_DURATION_MAX_SEC = 8.0     # safety timeout
POS_TOL    = 0.04               # meters (overall distance tolerance)
DEAD_BAND  = 0.01               # per-axis deadband [m] => treat as 0 error
KP_LIN     = 0.4                # smaller P gain to reduce overshoot

# Added 'h' for go-home
KEYS_OF_INTEREST = set("wasdqezijkluo,.h")

key_state = {k: False for k in KEYS_OF_INTEREST}
stop_flag = False

open_gripper_requested  = False
close_gripper_requested = False
help_requested          = False
home_requested          = False

# Rotation TOOL -> BASE
R_BT = np.eye(3)
R_lock = threading.Lock()

# Latest EE position in BASE (from /kinova/pose_tool_in_base)
ee_pos = None
ee_lock = threading.Lock()

# Homing state
home_active = False
home_start_time = None


def disable_terminal_echo():
    if not sys.stdin.isatty():
        return None
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    new_settings = termios.tcgetattr(fd)
    new_settings[3] &= ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
    return old_settings


def restore_terminal_settings(old_settings):
    if old_settings is None:
        return
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    termios.tcflush(fd, termios.TCIFLUSH)


def print_help():
    rospy.loginfo("")
    rospy.loginfo("Keyboard teleop: linear in BASE, angular in TOOL")
    rospy.loginfo("cmd = [vx_B, vy_B, vz_B, wx_B, wy_B, wz_B]")
    rospy.loginfo("---------------------------------------")
    rospy.loginfo("Linear (BASE frame):")
    rospy.loginfo("  w / s : +Y / -Y")
    rospy.loginfo("  a / d : +X / -X")
    rospy.loginfo("  q / e : +Z / -Z")
    rospy.loginfo("")
    rospy.loginfo("Angular (TOOL frame):")
    rospy.loginfo("  i / k : +roll / -roll (X_tool)")
    rospy.loginfo("  j / l : +pitch / -pitch (Y_tool)")
    rospy.loginfo("  u / o : +yaw / -yaw (Z_tool)")
    rospy.loginfo("")
    rospy.loginfo("Gripper (Float64MultiArray, len=1):")
    rospy.loginfo("  ,     : OPEN  (data = [1.0])")
    rospy.loginfo("  .     : CLOSE (data = [0.0])")
    rospy.loginfo("")
    rospy.loginfo("Home (position-only via velocity):")
    rospy.loginfo("  h     : go to home XYZ in BASE using internal velocity controller")
    rospy.loginfo("")
    rospy.loginfo("Other:")
    rospy.loginfo("  ?     : print this help")
    rospy.loginfo("  ESC   : quit teleop")
    rospy.loginfo("")


def on_press(key):
    global stop_flag, open_gripper_requested, close_gripper_requested, help_requested, home_requested
    try:
        ch = key.char
    except AttributeError:
        if key == keyboard.Key.esc:
            rospy.loginfo("ESC pressed, stopping teleop...")
            stop_flag = True
        return

    if ch in key_state:
        key_state[ch] = True

    if ch == ',':
        open_gripper_requested = True
    elif ch == '.':
        close_gripper_requested = True
    elif ch == '?':
        help_requested = True
    elif ch == 'h':
        home_requested = True


def on_release(key):
    try:
        ch = key.char
    except AttributeError:
        return
    if ch in key_state:
        key_state[ch] = False


def ee_pose_callback(msg: PoseStamped):
    """
    Update R_BT (TOOL -> BASE) and ee_pos from pose_tool_in_base.
    """
    global R_BT, ee_pos
    q = msg.pose.orientation
    quat = [q.x, q.y, q.z, q.w]
    M = tft.quaternion_matrix(quat)
    R = M[0:3, 0:3]
    with R_lock:
        R_BT = R

    with ee_lock:
        ee_pos = np.array([msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z], dtype=float)


def build_linear_base_from_keys():
    """
    Build linear velocity in BASE frame from keys.
    """
    vx = vy = vz = 0.0

    if key_state.get('w', False):
        vy -= LIN_SPEED
    if key_state.get('s', False):
        vy += LIN_SPEED
    if key_state.get('a', False):
        vx += LIN_SPEED
    if key_state.get('d', False):
        vx -= LIN_SPEED
    if key_state.get('q', False):
        vz += LIN_SPEED
    if key_state.get('e', False):
        vz -= LIN_SPEED

    return np.array([vx, vy, vz], dtype=float)


def build_angular_tool_from_keys():
    """
    Build angular velocity in TOOL frame from keys.
    """
    wx_T = wy_T = wz_T = 0.0

    if key_state.get('i', False):
        wx_T += ANG_SPEED
    if key_state.get('k', False):
        wx_T -= ANG_SPEED
    if key_state.get('j', False):
        wy_T += ANG_SPEED
    if key_state.get('l', False):
        wy_T -= ANG_SPEED
    if key_state.get('u', False):
        wz_T += ANG_SPEED
    if key_state.get('o', False):
        wz_T -= ANG_SPEED

    return np.array([wx_T, wy_T, wz_T], dtype=float)


def teleop_loop():
    global stop_flag, open_gripper_requested, close_gripper_requested
    global help_requested, home_requested, home_active, home_start_time

    rospy.init_node("kinova_cartesian_teleop_mixed_frame")

    vel_topic       = rospy.get_param("~vel_topic",       "/kinova_demo/cart_vel_cmd")
    vel_sel_topic   = rospy.get_param("~vel_sel_topic",   "/kinova_demo/cart_vel_sel")
    grip_topic      = rospy.get_param("~gripper_topic",   "/siemens_demo/gripper_cmd")
    ssa_enable_top  = rospy.get_param("~ssa_enable_topic","/siemens_demo/ssa_enable")
    ee_pose_topic   = rospy.get_param("~ee_pose_topic",   "/kinova/pose_tool_in_base")

    rospy.loginfo("Subscribing EE pose from:              %s", ee_pose_topic)
    rospy.loginfo("Publishing BASE-frame vel to:          %s", vel_topic)
    rospy.loginfo("Publishing selection mask to:          %s", vel_sel_topic)
    rospy.loginfo("Publishing gripper commands to:        %s", grip_topic)
    rospy.loginfo("Publishing SSA enable (if used) to:    %s", ssa_enable_top)
    rospy.loginfo("Home position (BASE) set to:           %s", HOME_POS_BASE.tolist())

    rospy.Subscriber(ee_pose_topic, PoseStamped, ee_pose_callback, queue_size=1)
    pub_vel      = rospy.Publisher(vel_topic,       Float64MultiArray, queue_size=10)
    pub_vel_sel  = rospy.Publisher(vel_sel_topic,   Float64MultiArray, queue_size=10)
    pub_grip     = rospy.Publisher(grip_topic,      Float64MultiArray, queue_size=10)
    pub_ssa      = rospy.Publisher(ssa_enable_top,  Bool,              queue_size=1)

    rate = rospy.Rate(LOOP_HZ)
    print_help()

    # Enable SSA once if used by driver
    try:
        rospy.loginfo("Enabling SSA / velocity mode (if used by driver)...")
        pub_ssa.publish(Bool(data=True))
    except Exception:
        pass

    while not rospy.is_shutdown() and not stop_flag:
        # --- One-shot actions ---

        if open_gripper_requested:
            open_gripper_requested = False
            msg = Float64MultiArray()
            msg.data = [1.0]   # OPEN
            rospy.loginfo("Gripper OPEN: %s", msg.data)
            pub_grip.publish(msg)

        if close_gripper_requested:
            close_gripper_requested = False
            msg = Float64MultiArray()
            msg.data = [0.0]   # CLOSE
            rospy.loginfo("Gripper CLOSE: %s", msg.data)
            pub_grip.publish(msg)

        if help_requested:
            help_requested = False
            print_help()

        # --- Start homing if requested ---

        if home_requested and not home_active:
            home_requested = False
            home_active = True
            home_start_time = rospy.Time.now()
            rospy.loginfo("Entering HOMING mode to position (BASE): %s", HOME_POS_BASE.tolist())

        # --- HOMING MODE: simple P controller on XYZ via velocity ---

        if home_active:
            with ee_lock:
                pos = None if ee_pos is None else ee_pos.copy()

            if pos is None:
                rospy.logwarn_throttle(1.0, "HOMING: EE position not yet available; sending zero velocity.")
                v_B = np.zeros(3, dtype=float)
            else:
                error = HOME_POS_BASE - pos
                dist  = np.linalg.norm(error)

                # Global tolerance on distance
                if dist < POS_TOL:
                    rospy.loginfo(
                        "HOMING: reached target (dist=%.4f < %.4f). Stopping.",
                        dist, POS_TOL
                    )
                    home_active = False
                    zero_msg = Float64MultiArray()
                    zero_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    pub_vel.publish(zero_msg)
                    rate.sleep()
                    continue

                # Per-axis deadband: treat small errors as zero to avoid dither
                for i in range(3):
                    if abs(error[i]) < DEAD_BAND:
                        error[i] = 0.0

                # Recompute distance after deadband, if everything is within deadband, stop
                if np.all(np.abs(error) < 1e-6):
                    rospy.loginfo(
                        "HOMING: inside per-axis deadband (|err_i| < %.3f). Stopping.",
                        DEAD_BAND
                    )
                    home_active = False
                    zero_msg = Float64MultiArray()
                    zero_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    pub_vel.publish(zero_msg)
                    rate.sleep()
                    continue

                # Safety timeout
                elapsed = (rospy.Time.now() - home_start_time).to_sec()
                if elapsed > HOME_DURATION_MAX_SEC:
                    rospy.logwarn("HOMING: timeout after %.2f s, stopping homing.", elapsed)
                    home_active = False
                    zero_msg = Float64MultiArray()
                    zero_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    pub_vel.publish(zero_msg)
                    rate.sleep()
                    continue

                # P-controller for velocity with clipping
                v_B = KP_LIN * error
                v_B = np.clip(v_B, -LIN_SPEED, LIN_SPEED)

                rospy.logdebug(
                    "HOMING: pos=%s, err=%s, v_B=%s, dist=%.4f",
                    pos.tolist(), error.tolist(), v_B.tolist(), dist
                )

            # During homing, angular velocities = 0
            vel_msg = Float64MultiArray()
            vel_msg.data = [v_B[0], v_B[1], v_B[2], 0.0, 0.0, 0.0]
            pub_vel.publish(vel_msg)

            sel_msg = Float64MultiArray()
            sel_msg.data = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            pub_vel_sel.publish(sel_msg)

            rate.sleep()
            continue  # skip normal teleop this cycle

        # --- NORMAL TELEOP (only when not homing) ---

        # 1) Linear in BASE
        v_B = build_linear_base_from_keys()

        # 2) Angular in TOOL, then rotate to BASE
        w_T = build_angular_tool_from_keys()
        with R_lock:
            R = R_BT.copy()
        w_B = R.dot(w_T)

        vel_msg = Float64MultiArray()
        vel_msg.data = [v_B[0], v_B[1], v_B[2], w_B[0], w_B[1], w_B[2]]
        pub_vel.publish(vel_msg)

        # Enable all DOFs
        sel_msg = Float64MultiArray()
        sel_msg.data = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        pub_vel_sel.publish(sel_msg)

        rate.sleep()

    # Zero on exit
    rospy.loginfo("Sending zero BASE-frame velocity and exiting.")
    zero_msg = Float64MultiArray()
    zero_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_vel.publish(zero_msg)


def main():
    old_tty = disable_terminal_echo()
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        teleop_loop()
    finally:
        listener.stop()
        restore_terminal_settings(old_tty)
    return 0


if __name__ == "__main__":
    sys.exit(main())
