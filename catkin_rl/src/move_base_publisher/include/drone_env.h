#ifndef DRONE_ENV_H
#define DRONE_ENV_H

#include <ros/ros.h>

bool is_within_radius(double target_x,double target_y,double radius);
bool check_goal(double x,double y);
double generate_random_coordinate(double min,double max);
void send_goal(double x,double y);
void publish_goal_marker(ros::Publisher& marker_pub,const geometry_msgs::PoseStamped& goal_pose);
bool spwanModelInGazebo(ros::NodeHandle &nh,const std::string &model_name,const std::string &model_file,const geometry_msgs::Pose &pose);
void spwanModelThread(ros::NodeHandle &nh,double x,double y);
bool deletModelInGazebo(ros::NodeHandle &nh, const std::string &model_name);
void updateModelInGazebo(ros::NodeHandle &nh,double x,double y);
#endif