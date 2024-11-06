# xtdrone_sec_dev

v1.0:正常悬停
     bug:有时需要打开多次

v1.1:communication.py增加/iris_0/communication,可以查看是否初始化完成
     bug:有时需要打开多次

v1.2:新增start.py可以一键执行communication.py和hover.py，效果可以悬停

v1.3:gazebo_env:新增暂停gazebo仿真,rviz可视化工具会显示在此

v1.4: Add 2d_motion_planning

TD3:神经网络文件夹
# 步骤：
启动PX4仿真

`roslaunch px4 indoor3.launch`

启动二维激光SLAM

`roslaunch hector_slam_launch hector_slam_xtdrone.launch`

建立和PX4仿真的通信，同时发布位置真值

`cd /home/ubuntu/lyl/xtdrone_sec_dev`
`python multirotor_communication.py iris 0`
`python3 communication.py`

将激光水平定位和高度真值数据通过MAVROS发给PX4

`cd ~/XTDrone/sensing/slam/laser_slam/script`
`python laser_transfer.py iris 0 hector`

启动运动规划

`roslaunch 2d_motion_planning.launch`


# 集群仿真
启动仿真

`cd ~/PX4_Firmware`
`roslaunch px4 multi_vehicle.launch`

建立通信

`cd ~/XTDrone/communication`
`bash multi_vehicle_communication.sh`

使用位姿真值

`cd ~/XTDrone/sensing/pose_ground_truth/`
`bash copy_get_multi_vehcle_local_pose.sh `

键盘控制起飞

`cd ~/XTDrone/control/keyboard`
`python3 multirotor_keyboard_control.py iris 4 vel`

转换相机姿态的方向

`cd ~/XTDrone/motion_planning/3d`
`python3 ego_swarm_transfer.py iris 4 `

启动rviz

`cd ~/XTDrone/motion_planning/3d`
`rviz -d ego_swarm_rviz.rviz`

启动ego_planner_swarm

`roslaunch ego_planner multi_uav.launch`

发布目的地

`cd ~/XTDrone/motion_planning/3d`
`bash ego_swarm_goal.sh`


强化学习： 增强对未知空间的探索
对比实验： 对比A*等算法的速度上的提升
试一下二维运动规划，ego_swarm.world:可以
？ego-规划器+move_base导航