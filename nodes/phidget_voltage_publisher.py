#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy

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
        arr = np.array([[12345, 0, np.random.random()],
                        [12345, 1, np.random.random()+10],
                        [12345, 2, np.random.random()+100], 
                        [34567, 0, np.random.random()+0], 
                        [34567, 1, np.random.random()+10], 
                        [34567, 2, np.random.random()+100], ])

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
