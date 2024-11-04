# xtdrone_sec_dev

v1.0:正常悬停
     bug:有时需要打开多次

v1.1:communication.py增加/iris_0/communication,可以查看是否初始化完成
     bug:有时需要打开多次

v1.2:新增start.py可以一键执行communication.py和hover.py，效果可以悬停

v1.3:gazebo_env:新增暂停gazebo仿真,rviz可视化工具会显示在此

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