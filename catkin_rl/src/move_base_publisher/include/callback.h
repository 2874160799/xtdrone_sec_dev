#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
#endif