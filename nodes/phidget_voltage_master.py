#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy
import time

# numpy imports - basic math and matrix manipulation
import numpy as np

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import MultiArrayDimension

import get_serial_numbers

################################################################################

################################################################################

class Phidget_Voltage_Publisher:
    def __init__(self, topic, serials):
        # Define the source of the images, e.g. rostopic name
        self.serials = serials # all the serial numbers you want to control
        print('Master control of these serials: ', serials)
        self.master_topic = topic + '_master'
        self.master_sub = rospy.Subscriber(self.master_topic, Float32, self.set_master)

        self.control_topic = topic
        self.val = 0
        
        # Raw Image Subscriber
        self.control_pub = rospy.Publisher(self.control_topic, Float32MultiArray)
        
    def set_master(self, master_msg):
        self.val = master_msg.data

    def pub_voltage(self):
        t = time.time()
        for serial in serials:
            arr = np.array([[serial, 0, self.val], # serial channel value
                            [serial, 1, self.val],
                            [serial, 2, self.val],
                            [serial, 3, self.val],])

        msg = Float32MultiArray(data=np.ravel(arr))

        d = MultiArrayDimension()
        d.label = "shape"
        d.size = arr.shape[0]
        d.stride = arr.shape[1]

        msg.layout.dim = [d]

        self.control_pub.publish(msg)

    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.pub_voltage()
            rate.sleep()
            
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='led',
                        help="ros topic with Float32 message for velocity control")
    parser.add_option("--serials", type="str", dest="serials", default=None,
                        help="list of serial numbers, e.g. [525330, 589935]. To find your serial, type lsusb -v and find your phidgets device(s) and look for iserial.")
    (options, args) = parser.parse_args()

    if options.serials is None:
        serials = get_serial_numbers.get_serial_numbers_for_idvend_idprod_name('06c2', '0037', 'Phidgets')
    else:
        serials = eval(serials)

    rospy.init_node('phidget_voltage_publisher', anonymous=True)
    phidget_voltage_publisher = Phidget_Voltage_Publisher(options.topic, serials)
    phidget_voltage_publisher.main()
