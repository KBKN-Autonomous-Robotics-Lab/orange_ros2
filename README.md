# orange_ros2 v0.2.0
Distribution: ROS2 Humble Hawksbill
## Setup
```
$ git clone https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2.git
$ rosdep install -r -y -i --from-paths .
```
## Launch simulation world
- empty_world
<img src="https://user-images.githubusercontent.com/84959376/211162608-ba114bec-af38-4f07-95ed-8c0dbecca21b.png" width="500px">

```
$ ros2 launch orange_gazebo empty_world.launch.xml
```
- orange_world
<img src="https://user-images.githubusercontent.com/84959376/211162925-7293f724-f4dd-422d-8253-d741626cc434.png" width="500px">

```
$ ros2 launch orange_gazebo orange_world.launch.xml
```
- orange_igvc
<img src="https://user-images.githubusercontent.com/84959376/211162991-aa3b2bfa-9334-4122-9f7d-2babdb99efc7.png" width="500px">

```
$ ros2 launch orange_gazebo orange_igvc.launch.xml
```
## Operate orange robot
- keyboard

```
$ ros2 launch orange_teleop teleop_keyboard.launch.xml
```
- gamepad

```
$ ros2 launch orange_teleop teleop_joy.launch.xml
```
## RViz2 visualization
<img src="https://user-images.githubusercontent.com/84959376/211379404-81bacf08-63d7-4fb6-bd76-46f2627bbe23.png" width="500px">

```
$ ros2 launch orange_bringup rviz2.launch.xml
```
## SLAM
<img src="https://user-images.githubusercontent.com/84959376/211379778-19499d00-f1b1-4cdb-a169-f7742f9317d6.png" width="300px">

### slam_toolbox

```
$ ros2 launch orange_slam slam_toolbox.launch.xml
```
