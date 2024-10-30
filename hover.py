import rospy
from geometry_msgs.msg import Twist,PoseStamped
import sys, select, os
import tty, termios
import time
from std_msgs.msg import String

forward = 0.0
leftward = 0.0
upward = 0.4
angular = 0.0
arm_flag = False
start_flag = True
time_flag = 1
hover_flag = False
run_once_flag = True


def pose_callback(msg):
  position = msg.pose.position
  if position.z > 1.0:
    global hover_flag 
    hover_flag = True
    #rospy.loginfo(f"Position - x:{position.x},y:{position.y},z:{position.z}")


if __name__ == "__main__":
  rospy.init_node('iris_0_hover',anonymous=True)
  cmd = String()
  twist = Twist()
  multi_cmd_vel_flu_pub = rospy.Publisher("/lyl/iris_0/cmd_vel_flu",Twist,queue_size=1)
  multi_cmd_pub = rospy.Publisher("/lyl/iris_0/cmd",String,queue_size=1)
  start_time = time.time()
  rospy.Subscriber('/iris_0/mavros/local_position/pose',PoseStamped,pose_callback)
  
  while not rospy.is_shutdown():  
    while time_flag :
      for i in range(3,0,-1):
        print(f"The Drone will Arming after {i}s")
        time.sleep(1)
      time_flag = 0
    if start_flag == 1 and time.time() - start_time > 3:
      start_flag = 0
      cmd = 'OFFBOARD'
      offboard_flag = False
      multi_cmd_pub.publish(cmd)
      print('offboard')
      arm_flag = True
    elif arm_flag == True:
        cmd = 'ARM'
        arm_flag = False
        multi_cmd_pub.publish(cmd)
        print('arming')
    if hover_flag == True and run_once_flag == True:
      cmd = 'HOVER'
      print('hover')
      multi_cmd_pub.publish(cmd)
      hover_flag = False
      run_once_flag = False
      forward = 0.0
      leftward = 0.0
      upward = 0.0
      angular = 0.0
    twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
    twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular
    multi_cmd_vel_flu_pub.publish(twist)
    multi_cmd_pub.publish(cmd)
    cmd = ''
    