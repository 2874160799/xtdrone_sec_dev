#include <ros/ros.h>
#include <send_goal.h>
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    send_goal(1.0,1.0,1.0);
    return 0;

}