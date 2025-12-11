#!/usr/bin/env python3

import sys
import os
import time
import threading
import termios

from pynput import keyboard

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

TIMEOUT_DURATION = 30.0
LIN_SPEED = 0.2   # m/s
ANG_SPEED = 20.0  # deg/s

# ------------ TTY helpers: disable terminal echo ------------

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


# ------------ Kinova helpers ------------

def check_for_end_or_abort(e):
    def _cb(notification, e=e):
        print("EVENT :", Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            e.set()
    return _cb


def set_single_level_servoing(base: BaseClient):
    servo = Base_pb2.ServoingModeInformation()
    servo.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(servo)


def example_move_to_home_position(base: BaseClient):
    set_single_level_servoing(base)

    print("Moving the arm to Home position...")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)

    home_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            home_handle = action.handle
            break

    if home_handle is None:
        print("Could not find Home action")
        return False

    e = threading.Event()
    notif = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(home_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notif)

    if not finished:
        print("Timeout while moving to Home")
        return False

    print("Home position reached.")
    return True


# ------------ Gripper helpers ------------

def send_gripper_position(base: BaseClient, position: float):
    """
    Send a single gripper position command.
    position in [0.0 (open), 1.0 (closed)]
    """
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_POSITION
    finger = cmd.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = position
    base.SendGripperCommand(cmd)


def open_gripper(base: BaseClient):
    print("Opening gripper...")
    send_gripper_position(base, 0.0)


def close_gripper(base: BaseClient):
    print("Closing gripper...")
    send_gripper_position(base, 1.0)


# ------------ Keyboard state ------------

# keys that affect twist / discrete actions
KEYS_OF_INTEREST = set("wasdqezijkluoh")  # h included for home

key_state = {k: False for k in KEYS_OF_INTEREST}
stop_flag = False
home_requested = False
help_requested = False
open_gripper_requested = False
close_gripper_requested = False


def on_press(key):
    global stop_flag, home_requested, help_requested
    global open_gripper_requested, close_gripper_requested

    try:
        ch = key.char
    except AttributeError:
        # Special keys
        if key == keyboard.Key.esc:
            print("ESC pressed, stopping teleop...")
            stop_flag = True
        return  # keep listener running

    # continuous keys: movement
    if ch in key_state:
        key_state[ch] = True

    # one-shot actions
    if ch == 'h':
        home_requested = True
    elif ch == '?':
        help_requested = True
    elif ch == '[':
        open_gripper_requested = True
    elif ch == ']':
        close_gripper_requested = True


def on_release(key):
    try:
        ch = key.char
    except AttributeError:
        return  # ignore special key releases

    if ch in key_state:
        key_state[ch] = False


# ------------ Help text ------------

def print_help():
    print("")
    print("Keyboard teleop (Twist / velocity mode)")
    print("---------------------------------------")
    print("Linear (base frame):")
    print("  w / s : +Y / -Y")
    print("  a / d : +X / -X")
    print("  q / e : +Z / -Z")
    print("")
    print("Angular (around tool axes):")
    print("  i / k : +roll / -roll (around X)")
    print("  j / l : +pitch / -pitch (around Y)")
    print("  u / o : +yaw / -yaw (around Z)")
    print("")
    print("Gripper:")
    print("  [     : open gripper")
    print("  ]     : close gripper")
    print("")
    print("Other:")
    print("  h     : go to Home position")
    print("  ?     : print this help")
    print("  ESC   : quit teleop")
    print("")


# ------------ Twist computation ------------

def build_twist_from_keys():
    """
    Map current key_state -> Twist (linear + angular velocities).
    """
    twist = Base_pb2.Twist()

    # Linear motion (base frame)
    if key_state.get('w', False):
        twist.linear_y += LIN_SPEED
    if key_state.get('s', False):
        twist.linear_y -= LIN_SPEED
    if key_state.get('a', False):
        twist.linear_x += LIN_SPEED
    if key_state.get('d', False):
        twist.linear_x -= LIN_SPEED
    if key_state.get('q', False):
        twist.linear_z += LIN_SPEED
    if key_state.get('e', False):
        twist.linear_z -= LIN_SPEED

    # Angular motion
    if key_state.get('i', False):
        twist.angular_x += ANG_SPEED
    if key_state.get('k', False):
        twist.angular_x -= ANG_SPEED
    if key_state.get('j', False):
        twist.angular_y += ANG_SPEED
    if key_state.get('l', False):
        twist.angular_y -= ANG_SPEED
    if key_state.get('u', False):
        twist.angular_z += ANG_SPEED
    if key_state.get('o', False):
        twist.angular_z -= ANG_SPEED

    return twist


def teleop_twist_loop(base: BaseClient):
    """
    Main streaming loop: at ~50 Hz, send a TwistCommand based on current key_state.
    Robot moves continuously while keys are held.
    """
    global stop_flag, home_requested, help_requested
    global open_gripper_requested, close_gripper_requested

    set_single_level_servoing(base)

    cmd = Base_pb2.TwistCommand()
    cmd.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE

    rate_hz = 50.0
    dt = 1.0 / rate_hz

    print_help()

    while not stop_flag:
        # Handle one-shot actions
        if home_requested:
            print("Home requested by key 'h'...")
            home_requested = False
            example_move_to_home_position(base)

        if help_requested:
            help_requested = False
            print_help()

        if open_gripper_requested:
            open_gripper_requested = False
            open_gripper(base)

        if close_gripper_requested:
            close_gripper_requested = False
            close_gripper(base)

        twist = build_twist_from_keys()
        cmd.twist.CopyFrom(twist)
        cmd.duration = 0   # int in your API

        base.SendTwistCommand(cmd)
        time.sleep(dt)

    # Send zero twist once when stopping
    zero = Base_pb2.Twist()
    cmd.twist.CopyFrom(zero)
    cmd.duration = 0
    base.SendTwistCommand(cmd)
    print("Teleop loop stopped, robot commanded to zero twist.")


# ------------ main() ------------

def main():
    # Import utilities (Kinova's helper module)
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    args = utilities.parseConnectionArguments()

    # Disable echo so keys aren't printed
    old_tty = disable_terminal_echo()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)  # kept for future extensions

        # Optional: go Home once at startup
        example_move_to_home_position(base)

        # Start keyboard listener in background
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        try:
            teleop_twist_loop(base)
        finally:
            listener.stop()
            restore_terminal_settings(old_tty)

    return 0


if __name__ == "__main__":
    sys.exit(main())
