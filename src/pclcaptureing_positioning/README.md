# Service node for moveIt!

## Introduction
In this folder you will find information and all the files about how to create a node that offers a service in ROS. This service allows you to request a position that will be sent to a moveIt movegroup.

In the rqt-graph screenshot you can see that the node is present. The node contains a service and is connected to movegroup through several topics. (/trajectory_execution_event, pickup/action_topics, place/action_topics, /planning_scene and /attached collision_object).
![Image of Graph](https://github.com/MinorAR/Fontys_SCARA_Arm/blob/master/img/rosgraph.png)

## how to use
As you can see in the picture below, the service is called with a rosservice call. You can enter the values you want to reach end the node will make sure the robot arm gets to this position. These values quaternion coordinates.
![Image of Servicecall](https://github.com/MinorAR/Fontys_SCARA_Arm/blob/master/img/srvcall.png)
You can see that the node sends an acknowlage back to the terminal.

## Folder contents
This is a catkin created package so the folders and most documents are automatically generated. also see: http://wiki.ros.org/catkin/Tutorials/create_a_workspace 

In the [src folder](http://github.com) I created a C++ node for the service call.

In the Launch folder I created a launch file to automatically launch all vital pats of the node.

In this folder you can find the Cmakelist which is ajusted to compile all the parts of this folder correct.
