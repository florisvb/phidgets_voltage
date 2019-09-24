#!/usr/bin/env python

# Command line arguments
from optparse import OptionParser

# ROS imports
import roslib, rospy

# numpy imports - basic math and matrix manipulation
import numpy as np

from std_msgs.msg import Float32MultiArray

import PhidgetVoltageClass

################################################################################
'''
This node controls the voltage of the analog voltage phidgets device.
Can simulaneously control multiple channels on multiple devices.

phidget_voltage_sinewave gives an example of how to control the voltage.
'''
################################################################################

class Phidget_Voltage_Control:
    def __init__(self, topic, default_value=0, min=0, max=5):
        # Define the topic that controls the voltage
        self.control_topic = topic
        self.control_sub = rospy.Subscriber(self.control_topic, Float32MultiArray, self.set_voltage)

        self.default_value = default_value
        self.max_value = 5
        self.min_value = 0

        self.devices = {}

    def initialize_device(self, serial):
        d = PhidgetVoltageClass.get_all_device_channels(serial)
        self.devices[serial] = d
        for channel in d:
            channel.setVoltage(self.default_value)
            channel.setEnabled(1)

    def set_voltage(self, control_msg):
        shape = [control_msg.layout.dim[0].size, control_msg.layout.dim[0].stride]
        arr = np.reshape(control_msg.data, shape)
        
        for row in arr:
            serial, channel, value = row
            serial = int(serial)

            if serial not in self.devices.keys():
                self.initialize_device(serial)

            channel = int(channel)
            if value > self.max_value:
                value = self.max_value
            if value < self.min_value:
                value = self.min_value
            self.devices[serial][channel].setVoltage(value)


    def main(self):
        rospy.spin()
            
################################################################################

if __name__ == '__main__':    
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                        help="ros topic with Float32 message for velocity control")
    parser.add_option("--default", type="float", dest="default", default=0,
                        help="default value for the voltage (between -10 and 10, default 0")
    (options, args) = parser.parse_args()

    rospy.init_node('phidget_voltage_control', anonymous=True)
    phidget_voltage_control = Phidget_Voltage_Control(options.topic, default_value=options.default)
    phidget_voltage_control.main()
