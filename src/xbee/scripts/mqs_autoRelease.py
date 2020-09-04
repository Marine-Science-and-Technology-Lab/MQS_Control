#!/usr/bin/env python
import rospy
import XBee
from time import sleep
from xbee.msg import auto_ctrl
from rospy.numpy_msg import numpy_msg

"""
*************************
This is a simple python script that reads a binary bite value from an XBee from labview
and triggers an automated driving function. The code is meant to be easily edited so that
different speeds for the driving motors or marine water jet can be added. Similarly with 
time delays for different functions
*************************
"""

go_mqs = False  # bool trigger
MET = 0  # mission elapsed time in seconds


# log auto message in the log
def callback(auto_ctrls):
    rospy.loginfo("Rx: %s", auto_ctrls)
    rospy.loginfo(type(auto_ctrls))


# looking for trigger from labview Xbee
def triggered():
    trigger = xbee.Receive()
    global go_mqs
    if trigger:
        content = trigger[7:-1]  # ignores first 8 bites of data to look at the actual message
        if content == 1:
            print("Trigger: " + xbee.format(content))
            rospy.loginfo("Trigger: " + xbee.format(content))
            go_mqs = True
        else:
            print("Wrong trigger recieved from labVIEW")
            print("Trigger: " + xbee.format(content))
            rospy.loginfo("Wrong trigger recieved from labVIEW...trigger: " + xbee.format(content))
    else:
        print("No trigger recieved from labVIEW")
        rospy.loginfo("No trigger recieved from labVIEW")


def xbee_pub():
    rospy.init_node('mqs_autoRelease', anonymous=True)
    while not go_mqs and rospy.is_shutdown():
        triggered()
    if go_mqs and not rospy.is_shutdown():
        # publish them controls
        rospy.Publisher("auto_ctrls", numpy_msg(auto_ctrl), mqs_auto_release)
        rospy.Publisher("auto_ctrls", numpy_msg(auto_ctrl), callback)

    rospy.spin()


def mqs_auto_release():
    # automated controls are published here
    """
    Automated control switches on once go_mqs is set to TRUE
    automated control sets the forward speed to 140 a crawl speed and sets the
    waterjet to full throttle
    Automated control will run until MET (mission elapsed time) reaches a predetermined
    time (more directly a number of cycles)
    """
    # auto_ctrls[1]=fwd
    # auto_ctrls[2]=rev
    # auto_ctrls[3]=marine throttle
    global MET
    rate = rospy.Rate(100)  # publish messages at 100Hz
    while not MET < 10 and rospy.is_shutdown():
        for i in auto_ctrl:
            # if i==0 or i==4:
            #    auto_ctrl.i=127 #keep steering centered
            if i == 1:
                auto_ctrl.i = 140  # slow forward crawl
            if i == 3:
                auto_ctrl.i = 255  # full marine throttle
            else:
                auto_ctrl.i = 0  # send centered or off signal
            rospy.Publisher.publish(auto_ctrl)
            rate.sleep()
            MET += 0.01  # 0.01 seconds per cycle since rate is 100Hz


if __name__ == "__main__":
    xbee = XBee.XBee("/dev/ttyUSB0")
    try:
        xbee_pub()
    except rospy.ROSInterruptException:
        pass
