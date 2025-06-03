# RobotLocalization Node Usage Guide

This guide provides concise instructions on how to:

- Install necessary dependencies
- Build the package
- Run the node
- Echo the pose topic
- Use the reset pose service

---

## Prerequisites

- ROS Installed: Ensure that ROS is properly installed on the system.
- Catkin Workspace: The `robotLocalization` package should be placed within the catkin workspace. In this guide, we assume the workspace is located at `~/workspace/pepper_rob_ws`.

---

## Package Location

The `robotLocalization` package should be located in:

```
~/workspace/pepper_rob_ws/src/cssr4africa/cssr_system/
```

The directory structure should include the following key files and directories:

- `CMakeLists.txt`
- `package.xml`
- `src/robotlocalization.cpp`
- `srv/SetPose.srv`
- `launch/robot_localization.launch`
- `config/robotLocalizationConfiguration.ini`
- `data/pepperTopics.dat`
- `data/simulatorTopics.dat`

---

## Step 1: Install Necessary Dependencies

Navigate to the catkin workspace and install any missing dependencies using `rosdep`:

```bash
cd ~/workspace/pepper_rob_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 2: Build the Package

Build the workspace to compile the `robotLocalization` package:

```bash
catkin_make
```

After building, source the workspace to update the environment:

```bash
source devel/setup.bash
```

---

## Step 3: Run the Node

To start the `robotLocalization` node, use the following command:

```bash
rosrun cssr_system robotLocalization
```

- Note: Ensure that the node name matches the executable specified in the `CMakeLists.txt`. In this case, the executable is named `robotlocalization`.

---

## Step 4: Echo the Pose Topic

The node publishes the robot's pose on the `/robotLocalization/pose` topic. To view the pose data in real-time:

```bash
rostopic echo /robotLocalization/pose 
```

This command displays the `x`, `y`, and `theta` (orientation) values of the robot's pose.

---

## Step 5: Use the Reset Pose Service

The `robotLocalization` node provides a service to reset the robot's pose to specified coordinates.

### Service Details

- Service Name: `/robotLocalization/reset_pose`
- Service Type: `robotLocalization/SetPose`

### Resetting the Pose

To reset the robot's pose to specific `x`, `y`, and `theta` values (with `theta` in degrees), execute:

Example:

```bash
rosservice call /robotLocalization/reset_pose "{x: 2.0, y: 3.0, theta: 90.0}"
```

This sets the robot's position to `(2.0, 3.0)` meters and orientation to `90` degrees.

Note: The `theta` value should be provided in degrees. Internally, it will be converted to radians.




Once the node is running, you can use the following commands to interact with it:
``` bash
rosservice call /robotLocalization/reset_pose 2.0 6.6 0.0
```

``` bash
br@br:~/workspace/pepper_rob_ws$ rosrun cssr_system robotNavigation 
```

``` bash
rosservice call /robotNavigation/set_goal 3.0 6.6 0.0
```
