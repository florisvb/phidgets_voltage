# phidgets_voltage
1. run: `rosrun phidgets_voltage phidget_voltage_control.py --topic='led'` 
2. run: `rosrun phidgets_voltage phidget_voltage_master.py` to controll all led's on all attached phidgets voltage controllers together
3. run: `rostopic pub /led_master std_msgs/Float32 "data: 0.0"` turns them on
4. run: `rostopic pub /led_master std_msgs/Float32 "data: 5.0"` turns them off
5. Important note: the software by defaults attempts to find all relevant serial numbers. But you may need to specify them manually if there are errors. See option `--serials` in phidget_voltage_master. 

# FAQ
Error about not finding `libphidget22.so.0.0.0`: assuming you installed libphidget22, you probably need to add `/usr/local/lib` to your path:
  * Add this to bashrc: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`
  * More info: https://www.phidgets.com/docs/OS_-_Linux
