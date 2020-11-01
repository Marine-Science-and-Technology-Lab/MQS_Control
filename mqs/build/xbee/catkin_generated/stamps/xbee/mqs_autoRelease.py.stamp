#!/usr/bin/env python
import rospy
import XBee
from time import sleep
from xbee.msg import auto_ctrl
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from collections import deque

"""
*************************
This is a simple python script that reads a binary bite value from an XBee from labview
and triggers an automated driving function. The code is meant to be easily edited so that
different speeds for the driving motors or marine water jet can be added. Similarly with 
time delays for different functions
*************************
"""

go_mqs = False  # bool trigger
auto_reset = False  # bool value for reseting auto_count in mqs_handshake
MET = 0  # mission elapsed time in seconds


# log auto message in the log
def callback(auto_ctrls):
    rospy.loginfo("Rx: %s", auto_ctrls)
    rospy.loginfo(type(auto_ctrls))


# looking for trigger from labview Xbee
def triggered():
    Msg = xbee.Receive()
    global go_mqs
    global auto_reset
    if Msg:
        content = Msg[9:-1]  # ignores first 10 bites of data to look at the actual message
        temp = xbee.format(content)  # converts hex to string
        if len(temp) == 2: #restricts messages that are not XX out
            trigger = int(temp,16)  # converts string to base 10 int
            if trigger == 1:
                print("Trigger: " + temp)
                rospy.loginfo("Trigger: " + temp)
                go_mqs = True
                go_pub = rospy.Publisher("go_mqs", Bool, queue_size=1)
                go_pub.publish(go_mqs)
                auto_reset = False
                xbee.RxMessages.clear()  # clears the queue on the recieving xbee
            else:
                print("Wrong trigger recieved from lavVIEW")
                print("Trigger recieved: " + temp)
                xbee.RxMessages.clear()
        else:
            print("Wrong trigger recieved from labVIEW")
            print("Trigger recieved: " + temp)
            rospy.loginfo("Wrong trigger recieved from labVIEW...trigger: " + temp)
            xbee.RxMessages.clear() #clears the queue on the recieving xbee


def xbee_pub():
    global go_mqs
    global auto_reset
    while not go_mqs and not rospy.is_shutdown():
        rospy.loginfo("Waiting for trigger from labVIEW")
        triggered()
    while go_mqs and not auto_reset and not rospy.is_shutdown():
        # publish them controls
        print("Automatic control Broadcasting...")
        rospy.loginfo("Automatic control Broadcasting...")
        mqs_auto_release()
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
    print("Automatic Control Enabled...")
    rospy.loginfo("Automatic Control Enabled...")
    auto_pub = rospy.Publisher('auto_ctrls', numpy_msg(auto_ctrl), queue_size=1)
    met_pub = rospy.Publisher('MET',Float32, queue_size=1)
    go_pub = rospy.Publisher("go_mqs", Bool, queue_size=1)
    auto_land_drive_ramp=0.1 #ramp motor up over 0.1 seconds
    auto_ctrls = auto_ctrl()
    auto_ctrls_ = []
    auto_ctrls_.extend(auto_ctrls.auto_ctrls)
    global MET
    global auto_reset
    global go_mqs
    rate = rospy.Rate(100)  # publish messages at 100Hz
    while MET < rospy.get_param("MET") and not rospy.is_shutdown():
        count = 0
        for i in range(len(auto_ctrls_)):
            if i == 0 and MET < rospy.get_param("auto_marine_steer_time"):
                auto_ctrls_[i]=rospy.get_param("auto_marine_steer")
                if auto_ctrls_[i]<0 or auto_ctrls_[i]>255:
                    auto_ctrls_[i]=127 #center jet
                    print("Invalid range for marine steering!")
                print("Marine steering set to ", auto_ctrls_[i])
            elif i == 1 and MET <= rospy.get_param("auto_land_drive_time"): #swap to > for an inbound run [also called delta time]
                if MET < auto_land_drive_ramp:
                    auto_ctrls_[i]=160 #crawl speed for ramp time of 0.2 seconds
                else:
                    auto_ctrls_[i]=rospy.get_param("auto_land_drive")   # set auto controls to the specified auto_land_drive parameter value
                if auto_ctrls_[i] <= 127 or auto_ctrls_[i] > 255: #if the value read from the server was not set or is invalid (including reverse) use a default forward crawl value of 160.
                    auto_ctrls_[i] = 160
                    print("Land drive parameter not set, land drive set to default forward crawl")
                print("Drive Wheels set to: ", auto_ctrls_[i])
            elif i == 1 and MET >= rospy.get_param("auto_land_drive_time"): #swap to < for an inbound run [also called delta time]
                auto_ctrls_[i] = 127  # stop driving after MET reaches the set parameter time in seconds cause you're in the water babyyyyyyy
                print("Drive Wheels stopped!")
            elif i == 2:
                auto_ctrls_[i] = rospy.get_param("auto_marine_drive")  # set marine throttle to the specified auto_marine_drive parameter value
                if auto_ctrls_[i] <= 0 or auto_ctrls_[i] > 255: #if the value read from the server was not set or is invalid use a default throttle value of 10%
                    auto_ctrls_[i] = 26
                    print("Marine drive parameter not set, marine drive set to default 10% throttle")
                print("Marine Throttle set to:", auto_ctrls_[i])
            elif i == 3 and MET < rospy.get_param("auto_land_steer_time"):
                auto_ctrls_[i]=rospy.get_param("auto_land_steer") #set steering to commmanded time
                if auto_ctrls_[i]<0 or auto_ctrls_[i]>255:
                    auto_ctrls_[i] = 127  #center wheels
                    print("invalid range for land steering!")
                print("Land steering set to ", auto_ctrls_[i])
            elif i == 3 and MET >= rospy.get_param("auto_land_steer_time"):
                auto_ctrls_[i] = 127 #recenter wheels after time
            elif i == 7 and MET >= rospy.get_param("auto_wheels_up_time"):
                auto_ctrls_[i] = 1
                print("Raising wheels...")
            elif i == 4:  # daq
                auto_ctrls_[i] = 1 #don't turn off the DAQ!
            elif i == 5: #bilge pump
                auto_ctrls_[i] = 1 #turn on the bilge pump because I'll probably forget
            elif i == 8: #cooling pump
                auto_ctrls_[i] = 1 #turn on the cooling pump because I'll probably forget
            else:
                auto_ctrls_[i] = 0  # send centered or off signal
            count += 1
        auto_pub.publish(auto_ctrls_)
        callback(auto_ctrls_)
        rate.sleep()
        MET += 0.01  # 0.01 seconds per cycle since rate is 100Hz
        print("MET: ", MET)
        met_pub.publish(MET)
    if MET >= rospy.get_param("MET"):
        print("Automatic Control Has Finished at MET: ", MET)
        # rospy.loginfo("Automatic Control Has Finished at MET: ",MET)
        auto_reset = True
        go_mqs = False  # send a trigger to handshake to reset the automatic count
        MET = 0  # reset the MET
        print("MET reset: ", MET)
        go_pub.publish(go_mqs)
        xbee_pub()  # might be circular


if __name__ == "__main__":
    xbee = XBee.XBee("/dev/ttyUSB0")
    rospy.init_node('mqs_autoRelease', anonymous=True)
    while not rospy.is_shutdown():
        xbee_pub()
