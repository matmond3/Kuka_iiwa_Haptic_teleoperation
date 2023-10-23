# Kuka_iiwa_Haptic_teleoperation
Our innovative platform uses ROS to teleoperate a KUKA iiwa robot with a Sigma 7 haptic device. The setup includes an RGBD camera and a QR code to detect the target object and its working area in the Kuka workspace. This ensures excellent flexibility for future applications.

The Project use the iiwPy3 lybrary developed by @Modi1987. More information can be found here:
https://github.com/Modi1987/iiwaPy3

The code is divided in 3 main platforms:

- **Kuka iiwa** -> run by kuka_test.py :
    ROS NODE in Python3 for the control of the iiwa LBR14R820.
    This node read the position of the End-Effector of an Haptic Device (Omni Geomagic Touch) and mirroring the same
    movements in the Joints space through a scaling factor which is specified in the YAML file in kuka/kuka_control/config.
    Once the position of the Haptic device is saved in a variable, this is used to compute the joint position of the robot
    using the function genInverseKinematics.py and it is finally sent to the kuka for the mirroring.
- **Haptic Device** -> automatically detected by omni_node.cpp:
    This ROS node create a virtual environment in Chai 3D and connect the virtual tool to both the end-effector of the Kuka
    iiwa and to the Haptic Device. Note that the Haptic device is automatically detected by the system
- **RGBD Camera**

## Note:
The file "**Dissertation.pfd**" gives a full overview of the project, the goals and our achivements
