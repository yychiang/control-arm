#include "ros/ros.h"
//#include "control_arm/DriveTheJoints.h"
//#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>



ros::Publisher joint_pub;


float cubic_poly(float a2, float t){
    
    return 0.75*a2*t*t-0.25*a2*t*t*t;
    //return a2* t /2.0; 
  
}

float cubic_poly_2(float a2, float t){
    
    return -4*a2+6*a2*t-2.25*a2*t*t+0.25*a2*t*t*t;
    //return a2* t /2.0; 
  
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "control_arm");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type "trajectory_msgs::JointTrajectory" 
    // on the robot actuation topic with a publishing queue size of 10

    //joint_pub = n.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory_point", 10);
    joint_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_trajectory_point", 10);

    
    //trajectory_msgs::JointTrajectory msg_joints;
    std_msgs::Float64MultiArray msg_joints;
   
    //msg_joints.header.stamp = ros::Time::now();
    msg_joints.data.resize(5);
    //msg_joints.points.resize(1);

    //msg_joints.points[0].positions.resize(4);
    //msg_joints.points[0].velocities.resize(4);
    
    //msg_joints.joint_names[0] ="joint1";
    //msg_joints.joint_names[1] ="joint2";
    //msg_joints.joint_names[2] ="joint3";
    //msg_joints.joint_names[3] ="joint4";
    

    while (ros::ok()) {    

        //float angle[] = {0, -1.5707, -1.05, 0.35, 0.70};
        float angle[] = {0, 0, -1.005, 0.249 ,0.638};
        //float angle[] = {0,0,0,0,0.35};
        
        for(int i=0; i<=1000; i=i+1){

            for (int j=0; j<5; j++){
                msg_joints.data[j]=cubic_poly(angle[j],i*2.0/1000);
                
            }


            
 
            joint_pub.publish(msg_joints);   

            ros::Duration(0.002).sleep();
        }

        
        for(int i=1000; i >= 0; i=i-1){

            for (int j=0; j<5; j++){
                msg_joints.data[j] = cubic_poly(angle[j],i*2.0/1000);
                
            }

            
  
            joint_pub.publish(msg_joints);   

            ros::Duration(0.002).sleep();
        
        }
        ros::Duration(2).sleep();
        
        /*
        for(int i=0; i <=100; i++){

            for (int j=0; j<5; j++){
                msg_joints.data[j] = cubic_poly_2(angle[j],2+i*2.0/100);
                
            }

            
  
            joint_pub.publish(msg_joints);   

            ros::Duration(0.02).sleep();
        
        }
        */


        ros::spinOnce();
    }    

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}
