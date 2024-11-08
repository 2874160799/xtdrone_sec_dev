#include <iostream>
#include <random>
#include <cmath>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <thread>
//#include "send_goal.h"
#include <geometry_msgs/PoseStamped.h>
#include "drone_env.h"
#include "callback.h"

double drone_x;
double drone_y;

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"train_node");
    ros::NodeHandle nh;
    //publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",0);
    
    //subscriber
    ros::Subscriber odom_sub = nh.subscribe("/iris_0/mavros/local_position/odom",10,odomCallback);
    
    double x,y;
    do
    {
      x = generate_random_coordinate(-10,10);
      y = generate_random_coordinate(-12,8);
      ROS_INFO("The goal coordinrate is (%.1f,%.1f)",x,y);
    } while (!check_goal(x,y));

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.orientation.w = 1.0;

    std::thread goal_thread(send_goal,x,y);
    std::thread spawn_thread(spwanModelThread,std::ref(nh),x,y);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        publish_goal_marker(marker_pub,goal_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    goal_thread.join();
    spawn_thread.join();
    
    return 0;

}


                                                                         

