# deep_nav_layers
A series of plug-ins to the ROS navigation stack to incorporate deep learning inputs.

This work provides a promising look at the potential to leverage both the power of robotics and deep learning alongside each other. This technique has several real-world applications such as detecting stairs, potholes or other hazards to robots in unstructured environments.


![ROS Pipeline for deep_nav_layers](https://github.com/NVIDIA-AI-IOT/deep_nav_layers/blob/master/images/nodes.png?raw=true)

Required Package:
- ROS Kinetic : Docker Container [dusty_nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)
- [NVIDIA-AI-IOT/caffe_ros](https://github.com/NVIDIA-AI-IOT/caffe_ros)

Following are the steps:

- On Terminal #1
  ```bash ~/jetson-UGV/tools/sim-mode.bash```

- On Terminal #2
```roslaunch jackal_navigation chameleon_nav.launch from_bag:=true```

- On Terminal #3
  ```rosrun rviz rviz -d ~/jetson-UGV/ros/jackal_viz/navigation.rviz```

- On Terminal #1
```rosbag play /ssd/lastDay.bag --clock```
