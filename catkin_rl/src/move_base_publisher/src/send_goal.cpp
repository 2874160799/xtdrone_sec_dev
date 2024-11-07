#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <send_goal.h>



void send_goal(double x,double y,double orientation_w)
{
    ROS_INFO("THIS IS SEND_GOAL_NODE");
    //创建move_base的action客户端
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO("waiting for move_base action server ... ");
    ac.waitForServer();

    //创建目标消息
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    //设置目标点的位置和朝向
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = orientation_w; //无旋转，保持朝向

    //发布目标点
    ROS_INFO("sending goal to move_base ... ");
    ac.sendGoal(goal);

    //等待机器人到达目标点
    ac.waitForResult();

    //获取执行结果
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("SUCCESSFULLY reached the goal !");
    }
    else
    {
        ROS_INFO("failed to reached the goal");
    }
}

void publish_goal_marker(ros::Publisher& marker_pub,const geometry_msgs::PoseStamped& goal_pose)
{
    //创建marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_arrow";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    //设置箭头和朝向
    marker.pose = goal_pose.pose;

    //设置箭头颜色和大小
    marker.scale.x = 0.5; //箭头长度
    marker.scale.y = 0.1; //箭头宽度
    marker.scale.z = 0.1; //箭头厚度
    marker.color.r = 0.0f;  // 红色
    marker.color.g = 0.0f;  // 无绿色
    marker.color.b = 1.0f;  // 无蓝色
    marker.color.a = 1.0f;  // 完全不透明

    marker_pub.publish(marker);
}

