#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

extern double drone_x;
extern double drone_y;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    drone_x = msg->pose.pose.position.x;
    drone_y = msg->pose.pose.position.y;
}
