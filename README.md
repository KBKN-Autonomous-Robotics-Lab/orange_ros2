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
<img src="https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/assets/88425011/88b06df7-79ef-4791-b530-ff3582f8cdb8" width="500px">

```
$ ros2 launch orange_bringup rviz2.launch.xml
```
## SLAM
The following SLAM methods can be run.
| slam_toolbox  | cartographer |
| ------------- | ------------- |
| <img src="https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/assets/88425011/c5d295ca-7304-4aa5-b4df-697301505d9d" width="200px"> | <img src="https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/assets/88425011/ba0b84e9-d291-4d64-a164-ea0d5bf2e10a" width="200px"> |

**Gazebo simulation without creating waypoints**
```
$ ros2 launch orange_gazebo orange_hosei.launch.xml
$ ros2 launch orange_slam {SLAM_METHOD_NAME}.launch.xml use_sim_time:=true
$ ros2 launch orange_teleop teleop_keyboard.launch.xml
```
**Gazebo simulation with creating waypoints**
```
$ ros2 launch orange_gazebo orange_hosei.launch.xml
$ ros2 launch orange_slam waypoint_slam.launch.xml slam_method:={SLAM_METHOD_NAME} use_sim_time:=true
$ ros2 launch orange_teleop teleop_keyboard.launch.xml
```
**ros2 bag**
```
$ ros2 launch orange_bringup with_ros2bag.launch.xml
$ ros2 bag play your_bag -r 3 --clock
$ ros2 launch orange_slam {SLAM_METHOD_NAME}.launch.xml with_ros2bag:=true
```
## Navigation2
You can try Navigation2 with a map created by slam_toolbox or cartographer.

**Gazebo simulation**

<img src="https://github.com/KBKN-Autonomous-Robotics-Lab/orange_ros2/assets/88425011/eb644f26-6122-451a-bb41-2db72f260b22" width="400">



**Waypoint Navigation**
```
$ ros2 launch orange_gazebo orange_hosei.launch.xml
$ ros2 launch orange_navigation waypoint_navigation.launch.xml use_sim_time:=true
```
## Sample dataset
You can download the ros2 bag obtained from orange_hosei.

```
$ wget --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=1T3eFLUSbdlLayyaVRCAf1Hfi8K1FMR4K' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1T3eFLUSbdlLayyaVRCAf1Hfi8K1FMR4K" -O ros2bag_orange_hosei.zip && rm -rf /tmp/cookies.txt
```



## Reference
- slam_toolbox : https://github.com/SteveMacenski/slam_toolbox
- cartographer : https://github.com/cartographer-project/cartographer_ros
- Navigation2 : https://github.com/ros-planning/navigation2
