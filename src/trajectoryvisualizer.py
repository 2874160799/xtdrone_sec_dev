import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class  TrajectoryVisualizer:
    def __init__(self):
        rospy.init_node('trajectory_visualizer',anonymous=True)
        
        self.path_pub = rospy.Publisher('/iris_0/trajectory',Path,queue_size=10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        rospy.Subscriber('/iris_0/mavros/local_position/pose',PoseStamped,self.pose_callback,)

    def pose_callback(self,data):
        pose_stamped = PoseStamped()
        pose_stamped.header = data.header  
        pose_stamped.pose = data.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)
    
    def run(self):
        rospy.spin()

if __name__=='__main__':
    visualizer = TrajectoryVisualizer()
    visualizer.run()