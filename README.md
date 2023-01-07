# orange_ros2
Distribution: ROS2 Humble Hawksbill
## Launch simulation world
- empty_world
<img src="https://user-images.githubusercontent.com/84959376/211162608-ba114bec-af38-4f07-95ed-8c0dbecca21b.png" width="500px">

```
$ ros2 launch orange_gazebo empty_world.launch.py
```
- orange_world
<img src="https://user-images.githubusercontent.com/84959376/211162925-7293f724-f4dd-422d-8253-d741626cc434.png" width="500px">

```
$ ros2 launch orange_gazebo orange_world.launch.py
```
- orange_world
<img src="https://user-images.githubusercontent.com/84959376/211162991-aa3b2bfa-9334-4122-9f7d-2babdb99efc7.png" width="500px">

```
$ ros2 launch orange_gazebo orange_igvc.launch.py
```
## Operate orange robot
- keyboard

```
$ ros2 launch orange_teleop teleop_keyboard.launch.py
```
- gamepad

```
$ ros2 launch orange_teleop teleop_joy.launch.py
```
## RViz2 visualization
<img src="https://user-images.githubusercontent.com/84959376/211163308-5fdca67e-702c-41f3-9fe2-11d8d5033efd.png" width="500px">

```
$ ros2 launch orange_bringup rviz2.launch.py
```
