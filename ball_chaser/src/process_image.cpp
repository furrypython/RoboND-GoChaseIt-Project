#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
	
    // Call the command_robot service and pass the requested joint angles
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }	
}

void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_counter = 0;
    int front_counter = 0;
    int right_counter = 0;
	
    // TODO: 
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i += 3) {
        int position_index = i % (img.width * 3) / 3;
	
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            if(position_index <= 265) {
		left_counter += 1;                
            }
            if(position_index > 265 && position_index <= 533) {
		front_counter += 1;               
            }
            if(position_index > 533) {
		right_counter += 1;                
            }
	}
    }
		
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    vector<int> position_counter{left_counter, front_counter, right_counter};
    int where_to_move = *max_element(position_counter.begin(), position_counter.end());

    // Depending on the white ball position, call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera.
    if (where_to_move == 0){
        drive_robot(0.0, 0.0); // This request brings my_robot to a complete stop
    }
    else if (where_to_move == left_counter) {
	drive_robot(0.0, -0.5);  // This request should drive my_robot left
    }
    else if (where_to_move == front_counter) {
        drive_robot(0.5, 0.0);  // This request drives my_robot robot forward
    }
    else if (where_to_move == right_counter) {
        drive_robot(0.0, 0.5); // This request drives my_robot right
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

