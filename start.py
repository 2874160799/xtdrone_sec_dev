import subprocess
import rospy
from std_msgs.msg import Bool

rospy.init_node('iris_0_start',anonymous=True)
process_a = subprocess.Popen(['python3', 'communication.py'])
process_b = subprocess.Popen(['python3', 'hover.py'])

