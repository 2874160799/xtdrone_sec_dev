#ifndef SEND_GOAL_H
#define SEND_GOAL_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

void send_goal(double x, double y, double orientation_w);
void publish_goal_marker(ros::Publisher& marker_pub,const geometry_msgs::PoseStamped& goal_pose);
bool spwanModelInGazebo(ros::NodeHandle &nh,const std::string &model_name,const std::string &model_file,const geometry_msgs::Pose &pose);
void spwanModelThread(ros::NodeHandle &nh);

#endif // SEND_GOAL_H
