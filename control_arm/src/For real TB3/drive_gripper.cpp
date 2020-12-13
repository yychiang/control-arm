// For a real TurtleBot3 Robot: Gripper Control

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

ros::Publisher gripper_pub;  //use /gripper_position topic


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "control_arm");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type "std_msgs::Float64MultiArray" 
    // on the robot gripper topic with a publishing queue size of 10

   
    gripper_pub = n.advertise<std_msgs::Float64MultiArray>("/gripper_position", 10);

    std_msgs::Float64MultiArray msg_gripper;
   
    msg_gripper.data.resize(1);
    

    while (ros::ok()) {    

        float angle1[] = {0};  //0 is open
        float angle2[] = {-1}; //-1 is close 

        

        msg_gripper.data[0] = angle1[0];
        gripper_pub.publish(msg_gripper);
        ros::Duration(2).sleep();

        msg_gripper.data[0] = angle2[0];
        gripper_pub.publish(msg_gripper);
        ros::Duration(2).sleep();

        //==============================================
        ros::Duration(2).sleep();
        ros::spinOnce();
    }    

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
