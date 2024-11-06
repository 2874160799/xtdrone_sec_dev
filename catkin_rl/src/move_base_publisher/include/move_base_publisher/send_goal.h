#ifndef SEND_GOAL_H
#define SEND_GOAL_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

void send_goal(double x, double y, double orientation_w);

#endif // SEND_GOAL_H
