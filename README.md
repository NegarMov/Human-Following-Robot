<h1 align="center">Human Following TurtleBot</h1>
<h6 align="center">Spring-2023 Robotics Course Project at Amirkabir University of Tech.</h6>


## Introduction
In this project, a TurtleBot3 Burger is programmed using the ROS framework to follow a human detected by a pretrained YOLO model. \
The `ImageProcessor` node inputs the robot's camera view to the YOLO model, resulting in an array of all the detected objects in the scene, alongside their bounding box specifications. The `Controller` node requests the bounding box of a particular object (in this case, a 'person') from this node via the `ObjectDetection` service. Using the position of the bounding box and its height, the robot indicates its moving direction and estimates its distance to the person. A threshold for the distance is set so that the robot will not get too close to the human and maintain a minimum distance. The linear speed of the robot is constant. However, a P-controller is adopted for the angular speed of the robot to ensure a quick response. If no human is detected in the scene, the robot stays in the same place and rotates to find one.

## Demo
Here you can observe the robot operating in the Gazebo simulation environment. The person is controlled using the arrow keys, and the TurtleBot is programmed to follow it as mentioned above.
<p align="center">
  <img src="https://github.com/NegarMov/Human-Following-Robot/blob/master/video/HumanFollower.gif" alt="Human following bot demo" width="500"/>
</p>
