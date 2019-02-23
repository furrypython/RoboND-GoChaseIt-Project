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
    
## Directory Structure  
### Directory Structure
```
    .GoChaseIt                         # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── myworld.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──                                                           
```
 
## Step to test `process_image`
### Step 1 Launch the robot inside the world 
```sh
$ cd (your workspace)/catkin_ws/  
$ source devel/setup.bash  
$ roslaunch my_robot world.launch
```
### Step 2 Run `drive_bot` and `process_image.cpp`  
```sh
$ cd (your workspace)/catkin_ws/  
$ source devel/setup.bash  
$ roslaunch ball_chaser ball_chaser.launch
```
Now place the white ball at different positions in front of the robot and see if the robot is capable of chasing the ball.

 
## References  
1. [The inertia matrix explained](http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/): GAZEBO ANSWERS  
2. [[ROS Projects] – Exploring ROS using a 2 Wheeled Robot](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/): The Construct  
3. [移動型ロボットのURDF作成](https://gbiggs.github.io/rosjp_urdf_tutorial_text/mobile_robot_urdf.html): ロボットモデリング講習会

  
