# phidgets_voltage
run: rosrun phidgets_voltage pdget_voltage_control.py --topic='led'
and: rosrun phidgets_voltage phidget_voltage_master.py
then: rostopic pub /led_master std_msgs/Float32 "data: 0.0" turns them on
and rostopic pub /led_master std_msgs/Float32 "data: 5.0" turns them off