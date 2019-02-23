# Go Chase It Project  
This is the 2nd project in the Udacity Nanodegree Course: Robotics Software Engineering.  
  
## Project Description  
In this project, I created two ROS packages inside my `catkin_ws/src`:  
1. `my_robot`:  
holds a robot designed with the Unified Robot Description Format, a white-colored ball, and a gazebo world.  
2. `ball_chaser`:  
contains two C++ ROS nodes (`drive_bot` & `process_image`) to interact with the robot and make it chase the white ball.  
    - `drive_bot`:  
    provides a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities.  
    The service publishes to the wheel joints and return back the requested velocities.  
    - `process_image`  
    reads my robot’s camera image, analyzes it to determine the presence and position of a white ball.  
    If a white ball exists in the image, my node requests a service via a client to drive the robot towards it.
 

## References  
1. [Inertia matrix calculation](http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/)

  