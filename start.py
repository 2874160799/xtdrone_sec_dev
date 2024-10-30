import subprocess
import rospy
from std_msgs.msg import Bool
runonce_flag = True

def communication_callback(msg):
    if msg.data == True:
        global runonce_flag
        if runonce_flag == True:
            print("Communication OK!")
            procesas_b = subprocess.Popen(['python3','hover.py'])
            runonce_flag = False
            procesas_b.wait()

rospy.init_node('iris_0_start',anonymous=True)

process_a = subprocess.Popen(['python3', 'communication.py'])
communication_sub = rospy.Subscriber("/iris_0/communication",Bool,communication_callback,queue_size=1)
# process_b = subprocess.Popen(['python3', 'hover.py'])
process_a.wait()
