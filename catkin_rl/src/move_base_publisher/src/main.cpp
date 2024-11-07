#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <thread>
#include <send_goal.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    double x = nh.param("goal_x",1.0);
    double y = nh.param("goal_y",1.0);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",0);
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = x;    
    goal_pose.pose.position.y = y;   
    goal_pose.pose.orientation.w = 1.0;//朝向为默认

    std::thread goal_thread(send_goal,x,y,1.0);
    std::thread spwan_thread(spwanModelThread,std::ref(nh));


    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        publish_goal_marker(marker_pub,goal_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    goal_thread.join();
    spwan_thread.join();
    return 0;
}