#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy
import time

# numpy imports - basic math and matrix manipulation
import numpy as np

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

################################################################################

################################################################################

class Phidget_Voltage_Publisher:
    def __init__(self, topic):
        # Define the source of the images, e.g. rostopic name
        self.control_topic = topic
        
        # Raw Image Subscriber
        self.control_pub = rospy.Publisher(self.control_topic, Float32MultiArray)
        
    def pub_voltage(self):
        t = time.time()
        arr = np.array([[525330, 0, 2.5 + 2.5*np.sin(0.5*t)],
                        [525330, 1, 2.5 + 2.5*np.sin(0.7*t+np.pi/3.)],
                        [525330, 2, 2.5 + 2.5*np.sin(0.9*t+np.pi/2.)],])

        msg = Float32MultiArray(data=np.ravel(arr))

        d = MultiArrayDimension()
        d.label = "shape"
        d.size = arr.shape[0]
        d.stride = arr.shape[1]

        msg.layout.dim = [d]

        self.control_pub.publish(msg)

    def main(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pub_voltage()
            rate.sleep()
            
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                        help="ros topic with Float32 message for velocity control")
    (options, args) = parser.parse_args()

    rospy.init_node('phidget_voltage_publisher', anonymous=True)
    phidget_voltage_publisher = Phidget_Voltage_Publisher(options.topic)
    phidget_voltage_publisher.main()
