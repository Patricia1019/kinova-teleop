The command for running recording data from teleoperation is:
```bash
rosbag record -O /your/file/path.bag \
    /camera/color/image_raw \
    /kinova/pose_tool_in_base \
    /kinova/current_joint_state \
    /siemens_demo/gripper_cmd
```
