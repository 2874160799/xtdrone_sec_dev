import rospy
from mavros_msgs.msg import PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String,Bool
from pyquaternion import Quaternion
import sys

rospy.init_node('iris_0_communication',anonymous=True)
rate = rospy.Rate(30)

class Communication:
    def __init__(self):
        self.current_position = None
        self.current_yaw = 0
        self.hover_flag = 0
        #坐标框架
        self.coordinate_frame = 1
        #目标运动
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.motion_type = 0
        self.flight_mode = None
        #任务
        self.mission = None
        self.last_cmd = None
        
        '''
        ros subscribers
        '''
        #来自mavros的位姿控制
        self.local_pose_sub = rospy.Subscriber("/iris_0/mavros/local_position/pose",PoseStamped,self.local_pose_callback,queue_size=1)
        #外部程序发布的控制指令
        self.cmd_sub = rospy.Subscriber("/lyl/iris_0/cmd",String,self.cmd_callback,queue_size=3)
        #外部程序发布的姿态控制
        self.cmd_pose_flu_sub = rospy.Subscriber("/lyl/iris_0/cmd_pose_flu",Pose,self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/lyl/iris_0/cmd_pose_enu",Pose,self.cmd_pose_enu_callback,queue_size=1)
        #外部程序发布的速度控制
        self.cmd_vel_flu_sub = rospy.Subscriber("/lyl/iris_0/cmd_vel_flu",Twist,self.cmd_vel_flu_callback, queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber("/lyl/iris_0/cmd_vel_enu",Twist,self.cmd_vel_enu_callback, queue_size=1)
        #外部程序发布的加速度控制
        self.cmd_accel_flu_sub = rospy.Subscriber("/lyl/iris_0/cmd_accel_flu",Twist,self.cmd_accel_flu_callback, queue_size=1)
        self.cmd_accel_enu_sub = rospy.Subscriber("/lyl/iris_0/cmd_accel_enu",Twist,self.cmd_accel_enu_callback, queue_size=1)
        '''
        ros 
        '''
        self.target_motion_pub = rospy.Publisher("/iris_0/mavros/setpoint_raw/local",PositionTarget,queue_size=1)
        self.communication_pub = rospy.Publisher("/iris_0/communication",Bool,queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy("/iris_0/mavros/cmd/arming",CommandBool)
        self.flightModeService = rospy.ServiceProxy("/iris_0/mavros/set_mode",SetMode)
        self.set_param_srv = rospy.ServiceProxy("/iris_0/mavros/param/set",ParamSet)
        
        rcl_except = ParamValue(4,0.0)
        self.set_param_srv('COM_RCL_EXCEPT',rcl_except)
        #self.publish_message_lyl(True)
        print("iris_0:communication initialized")
    
    def start(self):
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            self.publish_message_lyl(True)
            rate.sleep()
    #将话题中的位置信息提取到当前对象中
    def local_pose_callback(self,msg):
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)
        
    
    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if(self.motion_type == 0):
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        if(self.motion_type == 1):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW
        if(self.motion_type == 2):
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW

        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)

    def cmd_pose_enu_callback(self, msg):
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def cmd_vel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)  

    def cmd_vel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)    

    def cmd_accel_flu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def cmd_accel_enu_callback(self, msg):
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(ax=msg.linear.x,ay=msg.linear.y,az=msg.linear.z,yaw_rate=msg.angular.z)    
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.02 or abs(y)  > 0.02 or abs(z)  > 0.02 or abs(w)  > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()

    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return#不做任何事情

        elif msg.data == 'ARM':#起飞
            self.arm_state =self.arm()
            print("iris_0:"+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print("iris_0:"+": Armed "+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print("iris_0:"+msg.data)

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def q2yaw(self, q):
        if isinstance(q, Quaternion):#如果q的数据类型是四元数类型
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("iris_0: arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("iris_0: disarming failed!")
            return False

    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.current_position.x,y=self.current_position.y,z=self.current_position.z,yaw=self.current_yaw)
        print("iris_0:"+self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        elif self.flightModeService(custom_mode=self.flight_mode):
            print("iris_0:"+self.flight_mode)
            return True
        else:
            print("iris_0:"+self.flight_mode+"failed")
            return False

    def publish_message_lyl(self,value):
        msg = Bool()
        msg.data = value
        self.communication_pub.publish(msg)
        
if __name__ == '__main__':
    communication = Communication()
    communication.start()