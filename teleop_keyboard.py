#!/usr/bin/env python3

###
# Simple keyboard teleop for Kinova Gen3 using Kortex TCP API
#
# Controls end-effector in Cartesian space with small position / orientation steps.
#
#   Position:
#       w/s : +X / -X  (forward / backward in base frame)
#       a/d : +Y / -Y  (left / right)
#       q/e : +Z / -Z  (up / down)
#
#   Orientation (Euler XYZ in degrees):
#       i/k : +theta_x / -theta_x  (roll)
#       j/l : +theta_y / -theta_y  (pitch)
#       u/o : +theta_z / -theta_z  (yaw)
#
#   Other:
#       h   : move to Home position
#       x   : exit
#
###

import sys
import os
import threading
import termios
import tty
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

TIMEOUT_DURATION = 10.0      # seconds to wait for an action
LIN_STEP = 0.02              # 2 cm per key press
ANG_STEP = 5.0               # 5 deg per key press


# ---------- Helpers ----------

def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications."""
    def check(notification, e=e):
        print("EVENT :", Base_pb2.ActionEvent.Name(notification.action_event))
        if (notification.action_event == Base_pb2.ACTION_END or
                notification.action_event == Base_pb2.ACTION_ABORT):
            e.set()
    return check


def getch():
    """
    Read a single keypress from stdin (no need to press Enter).
    Works on Linux terminals.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def set_single_level_servoing(base: BaseClient):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)


def move_to_home(base: BaseClient):
    print("Moving the arm to Home action...")
    # Make sure the arm is in Single Level Servoing mode
    set_single_level_servoing(base)

    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)

    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle
            break

    if action_handle is None:
        print("[WARN] No action named 'Home' found on the controller.")
        return False

    e = threading.Event()
    notif_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notif_handle)

    if finished:
        print("Home position reached.")
    else:
        print("[WARN] Timeout waiting for Home action.")
    return finished

def send_cartesian_delta(base: BaseClient,
                         base_cyclic: BaseCyclicClient,
                         dx=0.0, dy=0.0, dz=0.0,
                         dtx=0.0, dty=0.0, dtz=0.0):
    """
    Read current tool pose from BaseCyclic and send a ReachPose action
    to move to a new pose = current_pose + delta.
    """
    # Get current pose from cyclic feedback
    feedback = base_cyclic.RefreshFeedback()
    cur_x = feedback.base.tool_pose_x
    cur_y = feedback.base.tool_pose_y
    cur_z = feedback.base.tool_pose_z
    cur_tx = feedback.base.tool_pose_theta_x
    cur_ty = feedback.base.tool_pose_theta_y
    cur_tz = feedback.base.tool_pose_theta_z

    target_x = cur_x + dx
    target_y = cur_y + dy
    target_z = cur_z + dz
    target_tx = cur_tx + dtx
    target_ty = cur_ty + dty
    target_tz = cur_tz + dtz

    print(f"Target pose: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}, "
          f"tx={target_tx:.1f}, ty={target_ty:.1f}, tz={target_tz:.1f}")

    # Make sure servoing mode is correct
    set_single_level_servoing(base)

    # Build an action
    action = Base_pb2.Action()
    action.name = "teleop_step"
    action.application_data = ""

    reach_pose = action.reach_pose  # this is a ConstrainedPose

    # Fill target pose (Pose message)
    reach_pose.target_pose.x = target_x
    reach_pose.target_pose.y = target_y
    reach_pose.target_pose.z = target_z
    reach_pose.target_pose.theta_x = target_tx
    reach_pose.target_pose.theta_y = target_ty
    reach_pose.target_pose.theta_z = target_tz

    # NOTE: In this API version:
    #   - target_pose has NO reference_frame field
    #   - ConstrainedPose has NO duration field
    # So we don't set either; defaults are used by the controller.

    e = threading.Event()
    notif_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteAction(action)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notif_handle)

    if not finished:
        print("[WARN] Timeout waiting for teleop step action.")


def print_help():
    print("\nKeyboard teleop controls:")
    print("  Translation:")
    print("    w / s : +X / -X")
    print("    a / d : +Y / -Y")
    print("    q / e : +Z / -Z")
    print("  Orientation (deg):")
    print("    i / k : +theta_x / -theta_x  (roll)")
    print("    j / l : +theta_y / -theta_y  (pitch)")
    print("    u / o : +theta_z / -theta_z  (yaw)")
    print("  Other:")
    print("    h     : go to Home action")
    print("    ?     : print this help")
    print("    x     : exit\n")


# ---------- Main ----------

def main():
    # Import utilities helper from sibling directory (same pattern as Kinova examples)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    import utilities

    args = utilities.parseConnectionArguments()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        print("Connected to robot.")
        set_single_level_servoing(base)

        # Optional: move to Home at start
        try:
            move_to_home(base)
        except Exception as e:
            print(f"[WARN] Could not move to Home at startup: {e}")

        print_help()
        print("Start teleop. Press 'x' to quit.")

        while True:
            key = getch()
            if key == "x":
                print("Exiting teleop.")
                break
            elif key == "w":
                send_cartesian_delta(base, base_cyclic, dx=LIN_STEP)
            elif key == "s":
                send_cartesian_delta(base, base_cyclic, dx=-LIN_STEP)
            elif key == "a":
                send_cartesian_delta(base, base_cyclic, dy=LIN_STEP)
            elif key == "d":
                send_cartesian_delta(base, base_cyclic, dy=-LIN_STEP)
            elif key == "q":
                send_cartesian_delta(base, base_cyclic, dz=LIN_STEP)
            elif key == "e":
                send_cartesian_delta(base, base_cyclic, dz=-LIN_STEP)
            elif key == "i":
                send_cartesian_delta(base, base_cyclic, dtx=ANG_STEP)
            elif key == "k":
                send_cartesian_delta(base, base_cyclic, dtx=-ANG_STEP)
            elif key == "j":
                send_cartesian_delta(base, base_cyclic, dty=ANG_STEP)
            elif key == "l":
                send_cartesian_delta(base, base_cyclic, dty=-ANG_STEP)
            elif key == "u":
                send_cartesian_delta(base, base_cyclic, dtz=ANG_STEP)
            elif key == "o":
                send_cartesian_delta(base, base_cyclic, dtz=-ANG_STEP)
            elif key == "h":
                move_to_home(base)
            elif key == "?":
                print_help()
            else:
                # Ignore unknown key, but you could print it for debugging
                pass

        return 0


if __name__ == "__main__":
    sys.exit(main())
