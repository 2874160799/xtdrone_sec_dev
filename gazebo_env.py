import rospy
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker,MarkerArray
from std_srvs.srv import Empty
import time

class  GazeboEnv:
    def __init__(self):
        rospy.init_node('iris_0_GazeboEnv',anonymous=True)
        
        self.path_pub = rospy.Publisher('/iris_0/trajectory',Path,queue_size=10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        rospy.Subscriber('/iris_0/mavros/local_position/pose',PoseStamped,self.pose_callback,)
        
        self.publisher = rospy.Publisher('/iris_0/visualization_marker',MarkerArray,queue_size=1)
        #self.publish_marker()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_marker)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)
        self.pause = rospy.ServiceProxy("gazebo/pause_physics",Empty)
    def pose_callback(self,data):
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header  
        pose_stamped.pose = data.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

    def run(self):
        rospy.spin()

    
    def publish_marker(self,event = None):
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CYLINDER#圆柱形标记
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 1
        marker.pose.position.y = 1
        marker.pose.position.z = 0
        
        markerArray.markers.append(marker)
        self.publisher.publish(markerArray)

if __name__=='__main__':
    visualizer = GazeboEnv()
    
    # for i in range(3,0,-1):
    #     print(i)
    #     time.sleep(1)
    # rospy.wait_for_service("/gazebo/pause_physics")
    # try:
    #     visualizer.pause()
    # except (rospy.ServiceException) as e:
    #     print("/gazebo/unpause_physics service call failed")
    # visualizer.pause()
    visualizer.run()