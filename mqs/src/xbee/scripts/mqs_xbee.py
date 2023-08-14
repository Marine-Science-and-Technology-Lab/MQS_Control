#!/usr/bin/env python
import rospy
import XBee
from time import sleep
from xbee.msg import mqs_ctrl
from rospy.numpy_msg import numpy_msg


#This node subscribes for data being changed on cmd_ctrl
#and publishes it to the XBEE. Data is coming in as
#a properly ordered bytearray in cmd_ctrl.msg

def callback(cmds):
    rospy.loginfo("Rx: %s", cmds)
    #if cmds.cmds(4)==1:
     #   rospy.loginfo('DAQ is On')
    #rospy.loginfo(type(cmds))

#sends data over the xbee
def xbee_send(cmds):
    #sends the data on the 16 channels
    channels_byte=bytearray()
    channels_byte.extend(cmds.cmds)
    #sent=xbee.Send(bytearray(channels_byte))
    xbee.Send(bytearray(channels_byte))

def mqs_xbee():
    rospy.init_node('mqs_xbee', anonymous=True)
    #subscribe to cmd_ctrl
    rospy.Subscriber("cmds",numpy_msg(mqs_ctrl),xbee_send) #numpy msg converts from message type to standard int used in python
    #for debugging
    rospy.Subscriber("cmds",numpy_msg(mqs_ctrl),callback)

    rospy.spin()

if __name__=="__main__":
    xbee=XBee.XBee("/dev/ttyUSB0")
    mqs_xbee()

