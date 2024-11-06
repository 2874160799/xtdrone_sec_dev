import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

rospy.init_node('iris_0_hecor_laser_transfer')

pose_pub = rospy.Publisher("iris_0/mavros/vision_pose/pose",PoseStamped,queue_size=1)

local_pose = PoseStamped()
local_pose.header.frame_id = 'map'
hector = PoseStamped()
height = 0

def hector_callback(data):
    global hector
    hector = data

def height_distance_callback(msg):
    global height
    height = msg.ranges[0]
    if(height == float("inf")):
        height = 0

def hector_slam():
    global local_pose,height
    pose2d_sub = rospy.Subscriber("iris_0/pose",PoseStamped,hector_callback,queue_size=1)
    rate = rospy.Rate(100)
    while True:
        local_pose = hector
        local_pose.pose.position.z = height
        pose_pub.publish(local_pose)
        rate.sleep()

if __name__=="__main__":
    height_distance_sub = rospy.Subscriber("iris_0/distance",LaserScan,height_distance_callback,queue_size=1)
    hector_slam()