import rospy
from geometry_msgs.msg import Twist
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

def getKey():
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
  if rlist:
    key = sys.stdin.read(1)
  else:
    key = ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key
if __name__ == "__main__":
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('iris_0_hover',anonymous=True)
  cmd = String()
  twist = Twist()
  multi_cmd_vel_flu_pub = rospy.Publisher("/lyl/iris_0/cmd_vel_flu",Twist,queue_size=1)
  multi_cmd_pub = rospy.Publisher("/lyl/iris_0/cmd",String,queue_size=1)
  start_time = time.time()

  while not rospy.is_shutdown():
    key = getKey()
    #if key == 'b' or time.time() - start_time > 5:
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
    twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
    twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular
    multi_cmd_vel_flu_pub.publish(twist)
    multi_cmd_pub.publish(cmd)
    cmd = ''

  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)