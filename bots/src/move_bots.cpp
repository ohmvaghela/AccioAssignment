#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "cmd_vel_publisher");
  ros::NodeHandle nh;

  // Create a publisher object
  ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 100);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 100);
  ros::Publisher pub3 = nh.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 100);
  ros::Publisher pub4 = nh.advertise<geometry_msgs::Twist>("/tb3_3/cmd_vel", 100);

  // Create the message object
  geometry_msgs::Twist msg;
  msg.linear.x = 0.05;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.05;

  // Publish the message repeatedly at a rate of 10 Hz
  ros::Rate rate(10);
  while (ros::ok())
  {
    pub1.publish(msg);
    pub2.publish(msg);
    pub3.publish(msg);
    pub4.publish(msg);
    rate.sleep();
  }

  return 0;
}