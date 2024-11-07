# 这是关于发布目标点和用于在rviz中显示的功能包文件夹
![Alt text](image.png)

# 操作流程
`roslaunch px4 ego_swarm_2d_lidar.launch`

`roslaunch hector_slam_launch hector_slam_xtdrone.launch`

`cd ~/XTDrone/communication`
`python3 multirotor_communication.py iris 0`

`cd ~/XTDrone/sensing/slam/laser_slam/script`
`python3 laser_transfer.py iris 0 hector`

`cd ~/XTDrone/motion_planning/2d/launch`
`roslaunch 2d_motion_planning.launch`

`cd ~/XTDrone/control/keyboard`
`python3 multirotor_keyboard_control.py iris 1 vel`

撞墙/碰到代价地图、奖励降低


bug：目标点的rviz图标没有，但是路径图标有，