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
extern bool loop_flag;
extern bool delete_flag;
int loop_times = 1;
int main(int argc, char** argv) 
{
    ros::init(argc,argv,"train_node");
    ros::NodeHandle nh;
    //publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",0);
    
    //subscriber
    ros::Subscriber odom_sub = nh.subscribe("/iris_0/mavros/local_position/odom",10,odomCallback);

    geometry_msgs::PoseStamped goal_pose;

    double x,y;
    ros::Rate loop_rate(10);
    while(loop_flag)
    {
        do
        {
        x = generate_random_coordinate(-10,10);
        y = generate_random_coordinate(-12,8);
        ROS_INFO("The goal coordinrate is (%.1f,%.1f)",x,y);
        } while (!check_goal(x,y));

        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = x;
        goal_pose.pose.position.y = y;
        goal_pose.pose.orientation.w = 1.0;

        //publish_goal_marker(marker_pub,goal_pose);
        //以1hz的频率调用publish_goal_marker
        ros::Timer timer = nh.createTimer(ros::Duration(1.0), 
            [&marker_pub, &goal_pose](const ros::TimerEvent&)
            {
                publish_goal_marker(marker_pub, goal_pose);
            });

        std::thread goal_thread(send_goal, x, y);
        
        std::thread spawn_thread(spwanModelThread,std::ref(nh),x,y,loop_times);//放置模型线程


        goal_thread.join();
        spawn_thread.join();
        
        //deletModelInGazebo(nh,"arrow_red_1");

        ros::spinOnce();
        loop_rate.sleep();
        loop_times++;
    }
    return 0;
}


                                                                         

