// For a real TurtleBot3 Robot

#include "ros/ros.h"
//#include "control_arm/DriveTheJoints.h"
#include <std_msgs/Float64MultiArray.h>



ros::Publisher joint_pub;
ros::Publisher gripper_pub;  //gripper_position


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

    // Inform ROS master that we will be publishing a message of type "std_msgs::Float64MultiArray" 
    // on the robot actuation topic with a publishing queue size of 10

   
    joint_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_trajectory_point", 10);
    gripper_pub = n.advertise<std_msgs::Float64MultiArray>("/gripper_position", 10);
    ros::Rate loop_rate(200);

    std_msgs::Float64MultiArray msg_joints;
    std_msgs::Float64MultiArray msg_gripper;
   
    //msg_joints.header.stamp = ros::Time::now();
    msg_joints.data.resize(5);
    msg_gripper.data.resize(1);
    

    while (ros::ok()) {    

        float angle1[] = {0, 0, -1.05, 0.35, 0.70};
        float angle2[] = {0, 0, -1.05, -0.9, -0.5};
        float angle3[] = {0, 0, 0, -0.4, 0};
        float angle4[] = {0, 0, -1, 0.3, 0.6};
        

        msg_gripper.data[0] = -1;
        gripper_pub.publish(msg_gripper);

        //================= path 1 =======================
        for(int i=0; i<=100; i=i+1){

            for (int j=0; j<5; j++){
                msg_joints.data[j] = cubic_poly(angle1[j],angle2[j],0,0,2.0,i*2.0/100);
            }
            

            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        }
        //------------------------------------------------

        //================= path 2 =======================
        for(int i=0; i<=100; i=i+1){

            for (int j=2; j<5; j++){
                msg_joints.data[j] = cubic_poly(angle2[j],angle3[j],0,2,1.0,i*1.0/100); //v2=1
            }
            //msg_gripper.data[0] = -1.5707;
            //gripper_pub.publish(msg_gripper);

            joint_pub.publish(msg_joints);   
            ros::Duration(0.01).sleep();
        }
        msg_gripper.data[0] = 0;
        gripper_pub.publish(msg_gripper);

        //================= path 3 =======================
        for(int i=0; i<=100; i=i+1){

            for (int j=2; j<5; j++){
                msg_joints.data[j] = cubic_poly(angle3[j],angle4[j],2,0,0.5,i*0.5/100); //v1=1
            }

            joint_pub.publish(msg_joints);   

            ros::Duration(0.005).sleep();
        }

        //================= path 4 =======================
        for(int i=0; i<=100; i=i+1){

            for (int j=0; j<5; j++){
                msg_joints.data[j] = cubic_poly(angle4[j],angle1[j],0,0,2.0,i*2.0/100);
            }

            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        }
        


        //==============================================
        ros::Duration(2).sleep();

        ros::spinOnce();
    }    

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
