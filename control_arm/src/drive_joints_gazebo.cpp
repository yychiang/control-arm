#include "ros/ros.h"
#include "control_arm/DriveTheJoints.h"
#include <trajectory_msgs/JointTrajectory.h>

ros::Publisher joint_pub;

/*
float cubic_poly(float a2, float t){
    
    return 0.75*a2*t*t-0.25*a2*t*t*t;
    //return a2* t /2.0; 
  
}
*/


// The following variables are:
// p1: position 1 (angle)
// p2: position 2
// v1: velocity 1
// v2: velocity 2
//  t: time
//  x: variable time
float cubic_poly(float p1, float p2, float v1, float v2, float t, float x){
   float a0,a1,a2,a3;
   a0 = p1;
   a1 = v1;
   a2 = (3/(t*t))*(p2-p1) - (2/t)*v1-(1/t)*v2;
   a3 = (-2/(t*t*t))*(p2-p1) + (1/(t*t))*(v1+v2);

   return a0+a1*x+a2*x*x+a3*x*x*x;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "control_arm");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type "trajectory_msgs::JointTrajectory" 
    // on the robot actuation topic with a publishing queue size of 10

    joint_pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);

    
    trajectory_msgs::JointTrajectory msg_joints;
    
   
    msg_joints.header.stamp = ros::Time::now();
    msg_joints.joint_names.resize(4);
    msg_joints.points.resize(1);

    msg_joints.points[0].positions.resize(4);
    msg_joints.points[0].velocities.resize(4);
    
    msg_joints.joint_names[0] ="joint1";
    msg_joints.joint_names[1] ="joint2";
    msg_joints.joint_names[2] ="joint3";
    msg_joints.joint_names[3] ="joint4";
    

    while (ros::ok()) {    

        float angle1[] = {0, -1.05, 0.35, 0.70};
        float angle2[] = {0, -1.05, -0.9, -0.5};
        float angle3[] = {0, 0, -0.4, 0};
        
        //================= path 1 =======================
        for(int i=0; i<=100; i++){

            for (int j=0; j<4; j++){
                msg_joints.points[0].positions[j] = cubic_poly(angle1[j],angle2[j],0,0,2.0,i*2.0/100);
                msg_joints.points[0].velocities[j] = 0;
            }


            msg_joints.points[0].time_from_start = ros::Duration(0.02);
 
            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        }

        //================= path 2 =======================
        for(int i=0; i<=100; i++){

            for (int j=0; j<4; j++){
                msg_joints.points[0].positions[j] = cubic_poly(angle2[j],angle3[j],0,0,2.0,i*2.0/100);
                msg_joints.points[0].velocities[j] = 0;
            }

            msg_joints.points[0].time_from_start = ros::Duration(0.02);
  
            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        
        }

        //================= path 3 =======================
        for(int i=0; i<=100; i++){

            for (int j=0; j<4; j++){
                msg_joints.points[0].positions[j] = cubic_poly(angle3[j],angle1[j],0,0,2.0,i*2.0/100);
                msg_joints.points[0].velocities[j] = 0;
            }


            msg_joints.points[0].time_from_start = ros::Duration(0.02);
 
            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        }



        
        msg_joints.points[0].time_from_start = ros::Duration(0.02);
        ros::spinOnce();
    }    

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
