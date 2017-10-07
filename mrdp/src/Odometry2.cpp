/*
 This code subscribes to 2 topics and publishes two things
 
 One subscriber subscribes to the node in the arduino arduino_odom
 The other subscribes to cmd_vel
 
 Cmd_vel will be published from a base station laptop
 This code will run on the MRDP laptop connected with arduino Mega

 The code publishes the transform baselink->odom required for Gmapping based on the odometry data it gets from arduino
 The code also publishes the pwm values for the wheels of the MRDP which will be subscribed in the arduino code PUBLSIHER NAME= arduino_vel
*/
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <stdlib.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#define width_robot 0.27  //distance between 2 wheels in mtrs
#define radius 0.047746   // radius of the wheel in mtrs


double vl = 0.0; // velocity of the wheels
double vr = 0.0;
class odometry
{
public:
	odometry();
private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub; 
	ros::Subscriber cmd_vel_sub; 
	ros::Publisher vel_pub; 
        ros::Publisher odom_pub;
	void callback_odom(const geometry_msgs::Vector3::ConstPtr& num);   
	void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& cmd_vel);				
};

//This function odometry data from the arduino and publishes the transform and odometry message
void odometry::callback_odom (const geometry_msgs::Vector3::ConstPtr& num)
{
	ros::Rate loop_rate(175);
	tf::TransformBroadcaster broadcaster;		
	geometry_msgs::TransformStamped odom_tran;
	geometry_msgs::Quaternion odom_quat;
	ros::Time current_time;
	
	current_time = ros::Time::now();
	
	// create/update transform

	odom_quat = tf::createQuaternionMsgFromYaw(num->z);
	odom_tran.header.frame_id = "odom"; //changed for the transform required by Gmapping
	odom_tran.child_frame_id = "base_link";
	odom_tran.header.stamp = current_time;
	odom_tran.transform.translation.x = num->x;  //num->x and y is the updated position of the bot from the arduino code
	odom_tran.transform.translation.y = num->y;
	odom_tran.transform.translation.z = 0.0;
	odom_tran.transform.rotation = odom_quat;
	
	//broadcast the required transform
        broadcaster.sendTransform(odom_tran);
    
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();// CHANGED THIS it was current_time. BUT THE WARNING DIDNT STOP
    odom.header.frame_id = "odom";

    //set the position
    // we set the position in the odom frame cos the point is in the world and not ON the robot!!! MIND=BLOWN
    odom.pose.pose.position.x = num->x;
    odom.pose.pose.position.y = num->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    // we set the velocity in the base_link frame because the velocity is wrt to the robot's frame which is the base_link frame
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x =(vl+vr)/2; 
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z =(vl-vr)/width_robot;
    
    odom_pub.publish(odom);
	
    
    loop_rate.sleep();	
}

//Based on cmd_vel commands this function publishes PWM values to left and right wheels of the MRDP
void odometry::callback_cmd_vel (const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	geometry_msgs::Vector3 arduino_vel; 
	double vel_x = cmd_vel->linear.x ;
	double vel_th = cmd_vel->angular.z;
	

	vr = vel_x + (vel_th * width_robot/2.0);
	vl = vel_x - (vel_th * width_robot/2.0);
	

	//finding the direction & the definitions for these values are present in the .ino file 

	if (vr>0 && vl>0)
      		arduino_vel.z = 1;   // forward  
	
        else if (vr<0 && vl<0)       // backward
      		{
                  arduino_vel.z = -1;
                  vr = -vr;
                  vl = -vl;
                }  
	
        else if (vr<0 && vl>0)       // right turn
		{
                  arduino_vel.z = 2;   
                  vr = -vr;
                } 
	else if (vr>0 && vl<0)      // left turn
                 { 
		arduino_vel.z = -2;  
                 vl = -vl;
                  }  
	//vr = abs(vr); //abs function does not work
	//vl = abs(vl);
     
	vr = vr*40.584525/radius; //velocity to PWM conversion
	vl = vl*40.584525/radius;

        if (vr>255) 	
		vr = 255;
	if (vr<0)
		vr = 0;
	if (vl>255)
		vl = 255;
	if (vl<0)
		vl = 0;
	
	arduino_vel.x = vr;
	arduino_vel.y = vl;
        //arduino_vel.z contains direction data	
	vel_pub.publish(arduino_vel);	
}


odometry::odometry() //constructor
{
	
	odom_sub = n.subscribe<geometry_msgs::Vector3>("/arduino_odom", 100, &odometry::callback_odom,this);
	cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 100, &odometry::callback_cmd_vel,this);
	vel_pub = n.advertise<geometry_msgs::Vector3>("arduino_vel",100); // publishes the velocities of the wheels and the direction
        odom_pub = n.advertise<nav_msgs::Odometry>("odom",100); 
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "odometry");
odometry k;
ros::spin();
}


	



