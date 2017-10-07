// the tf here is from /odom to /base_link but we need the exact opposite transform
//the vx and vy will need to be published from the encoders from the arduino

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv)

{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); //topic name is odom, publisher object is odom_pub
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0; 
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1; 
  double vy = 0.0;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0); 
  while(n.ok())
{

    ros::spinOnce();              
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt; 
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x; 
    y += delta_y;
    th += delta_th;

    
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now(); // was current_time i changed this
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;  // all these x,y, th are double data type

    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat; 
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();// CHANGED THIS it was current_time. BUT THE WARNING DIDNT STOP
    odom.header.frame_id = "odom";

    //set the position
    // we set the position in the odom frame cos the point is in the world and not ON the robot!!! MIND=BLOWN
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    // we set the velocity in the base_link frame because the velocity is wrt to the robot's frame which is the base_link frame
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
