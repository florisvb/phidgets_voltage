# run this inside an ipython terminal to get fast manual control of lights
# ctrl-copy
# %paste in ipython
import rospy
import time
from std_msgs.msg import Float32
rospy.init_node('phidget_test')
pub = rospy.Publisher('/led_master', Float32)
time.sleep(2)
# Turn off
pub.publish(5)
time.sleep(2)
# Turn on
pub.publish(0)
time.sleep(2)
# Partially on
pub.publish(2.5)
