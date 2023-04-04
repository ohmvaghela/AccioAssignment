#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "box_controller");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist vel;
    vel.linear.x = 1.0; // Move in the x direction at 1 m/s

    ros::Rate rate(10); // Publish at 10 Hz

    while (ros::ok())
    {
        pub.publish(vel);
        rate.sleep();
    }

    return 0;
}
