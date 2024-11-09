#ifndef DRONE_ENV_H
#define DRONE_ENV_H

#include <ros/ros.h>

bool is_within_radius(double target_x,double target_y,double radius);
bool check_goal(double x,double y);
bool checkModelExists(ros::NodeHandle &nh, const std::string &model_name);
bool deleteModelInGazebo(ros::NodeHandle &nh, const std::string &model_name);
void send_goal(double x,double y);
double generate_random_coordinate(double min,double max);
void publish_goal_marker(ros::Publisher& marker_pub,const geometry_msgs::PoseStamped& goal_pose);
bool spwanModelInGazebo(ros::NodeHandle &nh,const std::string &model_name,const std::string &model_file,const geometry_msgs::Pose &pose);
void spwanModelThread(ros::NodeHandle &nh,double x,double y,int i);
//bool deletModelInGazebo(ros::NodeHandle &nh, const std::string &model_name);


#endif