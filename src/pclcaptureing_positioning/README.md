# Service node for moveIt!

In this document you will find information about how to create a node that offers a service in ROS. This service allows you to request a position that will be sent to a moveIt movegroup.

In the rqt-graph screenshot you can see that the node is present. The node contains a service and is connected to movegroup through several topics. (/trajectory_execution_event, pickup/action_topics, place/action_topics, /planning_scene and /attached collision_object).
