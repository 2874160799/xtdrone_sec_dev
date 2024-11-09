#include <ros/ros.h>
#include <random>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <fstream>
#include <streambuf>
#include <nav_msgs/Odometry.h>
#include <cmath>


extern double drone_x;
extern double drone_y;

bool loop_flag = true;//开启新一轮训练的标志

std::string current_model_name = "";



bool is_within_radius(double target_x,double target_y,double radius)
{
    double distance = std::sqrt(std::pow(drone_x - target_x,2) + std::pow(drone_y - target_y,2)); 
    return distance <= radius;

}

//检查目标点是否在障碍物上
bool check_goal(double x,double y)//可能存在无法到达的点，需要修改
{
    bool goal_ok = false;
    if (-7 > x > -9.5 &&  7.5 > y > -1)
    goal_ok = true;
    if (-1.5 > x > -6 &&  7.5 > y > 2)
    goal_ok = true;
    if (-1.5 > x > -6 &&  0.5 > y > -1)
    goal_ok = true;
    if (-5 > x > -9.5 &&  -2.5 > y > -8)
    goal_ok = true;
    if (-5 > x > -9.5 &&  -9.5 > y > -11.5)
    goal_ok = true;
    if (2.5 > x > 0 &&  7.5 > y > -11.5)
    goal_ok = true;
    if (9.5 > x > 3.5 &&  7.5 > y > 5)
    goal_ok = true;
    if (6 > x > 3.5 &&  3 > y > -7)
    goal_ok = true;
    if (9.5 > x > 7 &&  3 > y > -7)
    goal_ok = true;
    if (9.5 > x > 3 &&  -8 > y > -11.5)
    goal_ok = true;
    if (-1 > x > -2.5 &&  -2.5 > y > -11.5)
    goal_ok = true;
    return goal_ok;
}
double generate_random_coordinate(double min,double max)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min,max);
    return dis(gen);
}

void send_goal(double x,double y)
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
    goal.target_pose.pose.orientation.w = 1.0; //无旋转，保持朝向

    //发布目标点
    ROS_INFO("sending goal to move_base ... ");
    ac.sendGoal(goal);

    //等待机器人到达目标点
    // ac.waitForResult();
    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        if (is_within_radius(x,y,1.0))
        {
            ROS_INFO("SUCCESSFULLY reached the goal !");
            ac.cancelGoal();
            loop_flag = true;
            break;
        }
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_INFO("failed to reached the goal");
            break;
        }
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

bool spwanModelInGazebo(ros::NodeHandle &nh,const std::string &model_name,const std::string &model_file,const geometry_msgs::Pose &pose)
{
    //创建服务客户端，连接到gazebo的spwan_model服务
    ros::ServiceClient spwan_model_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

    std::ifstream t(model_file);
    std::string model_xml((std::istreambuf_iterator<char>(t)),std::istreambuf_iterator<char>());

    //创建spwan_model 服务请求
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = model_name;
    srv.request.model_xml = model_xml;
    srv.request.initial_pose = pose;
    srv.request.reference_frame = "world";

    //调用服务
    if (spwan_model_client.call(srv))
    {
        ROS_INFO("Successfully spwaned model %s",model_name.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to spwan model %s",model_name.c_str());
        return false;
    }
}

void spwanModelThread(ros::NodeHandle &nh,double x,double y)
{
    //设置模型文件的名称，路径和位置
    std::string model_name = "arrow_red_1";
    std::string model_file = "/home/ubuntu/.gazebo/models/arrow_red/model.sdf";
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 5.0;
    //pose.orientation.w = 1.0;
    //设置四元数，绕y轴旋转90度
    pose.orientation.x = -sin(M_PI/4);
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = cos(M_PI/4);

    //调用spwanmodelingazebo函数来放置模型
    if (spwanModelInGazebo(nh,model_name,model_file,pose))
    {
        ROS_INFO("Model spwan sucessfully!");
    }
    else
    {
        ROS_ERROR("Failed to spwan model!");
    }
}

bool deletModelInGazebo(ros::NodeHandle &nh, const std::string &model_name)
{
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_srv;
    delete_srv.request.model_name = model_name;

    if (client.call(delete_srv))
    {
        if (delete_srv.response.success)
        {
            ROS_INFO("Model [%s] deleted successully!",model_name.c_str());
            return true;
        }
        else 
        {
            ROS_WARN("Model [%s] deletion failed: %s",model_name.c_str(),delete_srv.response.status_message.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call delete model service!");
    }
}

void updateModelInGazebo(ros::NodeHandle &nh,double x,double y)
{
    //如果上一个模型存在
    if(!current_model_name.empty())
    {
        deletModelInGazebo(nh,current_model_name);
    }
    //更新模型名字
    current_model_name = "arrow_red_" + std::to_string(ros::Time::now().toSec());
    //放置新的模型
    std::string model_file = "/home/ubuntu/.gazebo/models/arrow_red/model.sdf";
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 5.0;
    //pose.orientation.w = 1.0;
    //设置四元数，绕y轴旋转90度
    pose.orientation.x = -sin(M_PI/4);
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = cos(M_PI/4);

    if (spwanModelInGazebo(nh,current_model_name,model_file,pose))
    {
        ROS_INFO("Model [%s] spawned successfully at position (%.1f, %.1f)", current_model_name.c_str(), x, y);
    }
    else
    {
        ROS_ERROR("Failed to spawn model [%s]!", current_model_name.c_str());
    }
}
