import sys
import time 
from Phidget22.Devices.VoltageOutput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *

try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. Either add PhdiegtHelperFunctions.py to your project folder "
                      "or remove the import from your project.")
    sys.stderr.write("\nPress ENTER to end program.")
    readin = sys.stdin.readline()
    sys.exit()

'''
* Displays info about the attached Phidget channel.  
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param self The Phidget channel that fired the attach event
'''
def onAttachHandler(self):
    
    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information
        
        print("\nAttach Event:")
        
        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n    -> Channel Class: " + channelClassName + "\n    -> Serial Number: " + str(serialNumber) +
                "\n    -> Hub Port: " + str(hubPort) + "\n    -> Channel:  " + str(channel) + "\n")
        else:
            print("\n    -> Channel Class: " + channelClassName + "\n    -> Serial Number: " + str(serialNumber) +
                    "\n    -> Channel:  " + str(channel) + "\n")
        
    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
* Displays info about the detached Phidget channel.
* Fired when a Phidget channel with onDetachHandler registered detaches
*
* @param self The Phidget channel that fired the attach event
"""
def onDetachHandler(self):

    ph = self

    try:
        #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
        #www.phidgets.com/docs/Using_Multiple_Phidgets for information
        
        print("\nDetach Event:")
        
        """
        * Get device information and display it.
        """
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n    -> Channel Class: " + channelClassName + "\n    -> Serial Number: " + str(serialNumber) +
                "\n    -> Hub Port: " + str(hubPort) + "\n    -> Channel:  " + str(channel) + "\n")
        else:
            print("\n    -> Channel Class: " + channelClassName + "\n    -> Serial Number: " + str(serialNumber) +
                    "\n    -> Channel:  " + str(channel) + "\n")
        
    except PhidgetException as e:
        print("\nError in Detach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

"""
* Writes Phidget error info to stderr.
* Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
*
* @param self The Phidget channel that fired the attach event
* @param errorCode the code associated with the error of enum type ph.ErrorEventCode
* @param errorString string containing the description of the error fired
"""
def onErrorHandler(self, errorCode, errorString):

    sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")
            
"""
* Creates, configures, and opens a VoltageOutput channel.
* Provides interface for controlling Voltage of the VoltageOutput.
* Closes out VoltageOutput channel
*
* @return 0 if the program exits successfully, 1 if it exits with errors.
"""
def get_device_channel(serial_number, channel):
    try:
        """
        * Allocate a new Phidget Channel object
        """
        ch = VoltageOutput()

        """
        * Set matching parameters to specify which channel to open
        """
        #You may remove this line and hard-code the addressing parameters to fit your application
        
        ch.setDeviceSerialNumber(525330)
        ch.setIsHubPortDevice(False)
        ch.setChannel(channel)   
        
        """
        * Add event handlers before calling open so that no events are missed.
        """
        print("\n--------------------------------------")
        print("\nSetting OnAttachHandler...")
        ch.setOnAttachHandler(onAttachHandler)
        
        print("Setting OnDetachHandler...")
        ch.setOnDetachHandler(onDetachHandler)
        
        print("Setting OnErrorHandler...")
        ch.setOnErrorHandler(onErrorHandler)
        
        """
        * Open the channel with a timeout
        """
        print("\nOpening and Waiting for Attachment...")
        
        try:
            ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, ch)
            raise EndProgramSignal("Program Terminated: Open Failed")


        # ch.setVoltage(voltage)

        '''
        * Perform clean up and exit
        ''' 

        return ch

    except:
        pass


def get_all_device_channels(serial_number):
    device_channels = []
    for i in range(4):
        ch = get_device_channel(serial_number, i)
        device_channels.append(ch)
    return device_channels


'''
Looks like Python isn't looking in /usr/local/lib on Ubuntu.

Your options would be to either install libphidget22 in /usr/lib:
CODE: SELECT ALL

./configure --prefix=/usr && make && sudo make install

Or ensure Python is looking in /usr/local/lib, either in the current session:
CODE: SELECT ALL

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
python DigitalOutput.py

Or permanently with ldconfig:
CODE: SELECT ALL

Add /usr/local/lib to /etc/ld.so.conf
sudo ldconfig


'''