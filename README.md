# orange_ros2 [![](https://img.shields.io/badge/ROS2%20Humble-stable-green?style=flat-square&logo=ros)](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2)
This repository contains [ROS2](https://docs.ros.org/en/humble/index.html) version of Hosei orange, which has competed in the [Tsukuba Challenge](https://tsukubachallenge.connpass.com/) and [IGVC](http://www.igvc.org/). It supports several Gazebo world, SLAM methods and provides navigation feature.

<img src="https://user-images.githubusercontent.com/88425011/231134383-1ce0b5ae-ae6e-4078-87c2-fbce2280eca3.jpg" width="400px">

## Setup

```
$ git clone https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2.git
$ wstool merge orange_ros2/orange_ros2.rosinstall
$ wstool update
$ rosdep install -r -y -i --from-paths .
```
## Launch Gazebo world
The following 4 gazebo worlds can be run.
| empty_world  | orange_world | orange_igvc | orange_hosei |
| ------------- | ------------- | ------------- | ------------- |
| <img src="https://user-images.githubusercontent.com/88425011/231134704-b4ecbc8e-1a5a-4f35-b1e6-37c63cb2e6ed.jpg" width="200px"> | <img src="https://user-images.githubusercontent.com/88425011/232782465-8292dcb0-1a92-4ccf-b283-653ff7ae0243.jpg" width="200px"> | <img src="https://user-images.githubusercontent.com/84959376/215265550-11277797-dd37-4ce2-b4f2-2eb4be066d3d.png" width="200px"> | <img src="https://user-images.githubusercontent.com/88425011/232783327-9c4ce0dd-4cf5-4cde-a682-dfc72cae310c.jpg" width="200px"> |

```
$ ros2 launch orange_gazebo {GAZEBO_WORLD_NAME}.launch.xml
```
## Operate orange robot
Orange robot can be controlled via keyboard or gamepad.

<img src="https://user-images.githubusercontent.com/88425011/231137664-89adb063-e41b-4a9b-99be-15739ade3555.gif" width="400px">

**keyboard**

```
$ ros2 launch orange_teleop teleop_keyboard.launch.xml
```
**gamepad**

```
$ ros2 launch orange_teleop teleop_joy.launch.xml
```
## RViz2 visualization
<img src="https://user-images.githubusercontent.com/84959376/211379404-81bacf08-63d7-4fb6-bd76-46f2627bbe23.png" width="500px">

```
$ ros2 launch orange_bringup rviz2.launch.xml
```
## SLAM
The following SLAM methods can be run.
| slam_toolbox  | cartographer |
| ------------- | ------------- |
| <img src="https://user-images.githubusercontent.com/84959376/215267963-16a0476b-3edd-4d58-a067-a416555f5dec.gif" width="200px"> | <img src="https://user-images.githubusercontent.com/84959376/215267937-0203156d-dbe0-4d4e-a496-96c2c9ec00c2.gif" width="200px"> |

**Gazebo simulation**
```
$ ros2 launch orange_gazebo orange_world.launch.xml
$ ros2 launch orange_slam {SLAM_METHOD_NAME}.launch.xml
$ ros2 launch orange_teleop teleop_keyboard.launch.xml
```
**ros2 bag**
```
$ ros2 bag play your_bag -r 3
$ ros2 launch orange_slam {SLAM_METHOD_NAME}.launch.xml with_ros2bag:=true
```
## Navigation2
You can try Navigation2 with a map created by slam_toolbox or cartographer.

**Gazebo simulation**

<img src="https://user-images.githubusercontent.com/84959376/215110831-6732c2e6-726c-47e6-a272-a0c874311e3c.gif" width="400">

```
$ ros2 launch orange_gazebo orange_world.launch.xml
$ ros2 launch orange_navigation navigation2.launch.xml slam_method:={SLAM_METHOD_NAME}
```
## Sample dataset
You can download the ros2 bag obtained from orange_world.

- [ros2bag_orange_hosei.zip](https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/files/11496734/ros2bag_orange_hosei.zip)




## Reference
- slam_toolbox : https://github.com/SteveMacenski/slam_toolbox
- cartographer : https://github.com/cartographer-project/cartographer_ros
- Navigation2 : https://github.com/ros-planning/navigation2
