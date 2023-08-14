#!/usr/bin/env python
import rospy
import numpy
from xbee.msg import cmd_ctrl
from time import sleep
from xbee.msg import auto_ctrl
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Float32

"""
This is a simple user controlled function that will either make the MQS perform circular or zigzag manuevers 
for a given time (MET) or until turned off by the pilot. Circle or zig zag size may be changed on the parameter server

To run this code set:
MET to desired value
auto_marine_steer_time to desired value (can be the same as the MET)
auto_marine_Steer to desired value
zigzag set to desired operation

"""


# log auto message in the log
def autoCallback(auto_ctrls):
    rospy.loginfo("Tx: %s", auto_ctrls)


# log the trigger incoming message in the log
def triggerCallback(cmd_ctrls):
    rospy.loginfo("Rx: %s", cmd_ctrls)


# looking for trigger from XBOX controller
def maneuver_trigger(cmd_ctrls):
    global msg
    global prev_msg
    global go_mqs
    global auto_reset
    cmd_ctrls_ = bytearray()
    cmd_ctrls_.extend(cmd_ctrls.cmd_ctrls)
    triggerCallback(cmd_ctrls) # see what the incoming message for debugging
    if cmd_ctrls_:  # if the message is full
        temp = str(cmd_ctrls_[11])  # convert int to string so we can print diagnostics
        msg = int(temp, 16)  # start button zero or one
        if msg == 1 and prev_msg == 0:
            rospy.loginfo("Trigger: " + temp)
            go_mqs = True
            rospy.loginfo("Go MQS!")
            go_pub = rospy.Publisher("go_mqs", Bool, queue_size=2)
            go_pub.publish(go_mqs)
            auto_reset = False
        elif msg == 1 and prev_msg == 1:
            rospy.loginfo("Trigger not reset!")
        else:
            rospy.loginfo("Trigger not pressed")
            rospy.loginfo("Go MQS: " + str(go_mqs))
            rospy.loginfo("Reset: " + str(auto_reset))
            rospy.loginfo("Msg value %d" % msg)
            rospy.loginfo("prev msg %d" % prev_msg)
        prev_msg = msg



def maneuver_node_start(cmd_ctrls):
    global go_mqs
    global auto_reset
    # rospy.Subscriber("cmd_ctrls",numpy_msg(cmd_ctrl),triggerCallback)  #for debugging
    if not go_mqs and not rospy.is_shutdown():
        rospy.loginfo("Waiting for start from cmd_ctrl")
        maneuver_trigger(cmd_ctrls)
    if go_mqs and not auto_reset and not rospy.is_shutdown():
        # publish them controls
        rospy.loginfo("Automatic control Broadcasting...")
        mqs_maneuver()


def mqs_maneuver():
    # automated controls are published here
    """
    Automated control switches on once go_mqs is set to TRUE
    automated control only touches the nozzle direction and duration
    Automated control will run until MET (mission elapsed time) reaches a predetermined
    time (more directly a number of cycles)
    """
    # auto_ctrls[0]=marine steer
    global MET
    global auto_reset
    global go_mqs
    rospy.logwarn("Automatic Manuever Control Enabled...")
    auto_pub = rospy.Publisher('auto_ctrls', numpy_msg(auto_ctrl), queue_size=2)
    met_pub = rospy.Publisher('MET', Float32, queue_size=1)
    go_pub = rospy.Publisher("go_mqs", Bool, queue_size=1)
    #go_pub.publish(go_mqs)
    auto_ctrls = auto_ctrl()
    auto_ctrls_ = []
    auto_ctrls_.extend(auto_ctrls.auto_ctrls)

    zigzag_rate = rospy.get_param("zigzag_rate")
    zigzag = rospy.get_param("zigzag")
    print("Zigzag setting %d" % zigzag, " at rate %f" % zigzag_rate)
    t = 0.0  # initialize t
    rate = rospy.Rate(100)
    while MET < rospy.get_param("MET") and not rospy.is_shutdown():
        for i in range(len(auto_ctrls_)):
            if i == 0:  # marine steering
                if zigzag != 0 and MET < rospy.get_param("auto_marine_steer_time"):  # if the time is not set control is on joystick
                    if zigzag == 2:
                        rospy.loginfo("Circle Maneuver Active")
                        # do circles at specified steering param
                        auto_ctrls_[i] = rospy.get_param("auto_marine_steer")
                        if auto_ctrls_[i] < 0 or auto_ctrls_[i] > 255:
                            auto_ctrls_[i] = 127  # center jet
                            rospy.logwarn("Invalid range for marine steering!")
                    elif zigzag == 1:
                        rospy.loginfo("Zigzag Maneuver Active at rate %f" % zigzag_rate)
                        # do zigzags at specified rate
                        if t < zigzag_rate:
                            auto_ctrls_[i] = rospy.get_param("auto_marine_steer")
                            if auto_ctrls_[i] < 0 or auto_ctrls_[i] > 255:
                                auto_ctrls_[i] = 127  # center jet
                                rospy.logwarn("Invalid range for marine steering!")
                            print("Marine steering set to %d" % auto_ctrls_[i], " for t = %f" % t)
                        elif t >= zigzag_rate and t < 2 * zigzag_rate:
                            temp = 127 - rospy.get_param("auto_marine_steer")
                            auto_ctrls_[i] = 128 + temp  # to ensure mirroring about the cetner point at 127
                            rospy.loginfo("Temp: %d" % temp)
                            rospy.loginfo("auto steer: %d" % auto_ctrls_[i])
                            if auto_ctrls_[i] < 0 or auto_ctrls_[i] > 255:
                                auto_ctrls_[i] = 127  # center jet
                                rospy.logwarn("Invalid range for marine steering!")
                            print("Marine steering set to %d" % auto_ctrls_[i], " for t = %f" % t)
                        else:
                            t = 0  # reset t to start zigzag over
                            rospy.loginfo("t reset to 0")
                            auto_ctrls_[i] = rospy.get_param("auto_marine_steer")
                            rospy.loginfo("auto steer: %d" % auto_ctrls_[i])
                else:
                    rospy.logwarn("Zigzag or time parameter is set to 0 or was not set")  # log reason
                    auto_ctrls_[i] = 127  # keep the jet centered
            elif i == 1:
                auto_ctrls_[i] = 127  # for the love of god don't drive the wheels!
            elif i == 2:
                auto_ctrls_[i] = rospy.get_param(
                    "auto_marine_drive")  # set marine throttle to the specified auto_marine_drive parameter value
                if auto_ctrls_[i] < 0 or auto_ctrls_[i] > 255:
                    # if the value read from the server was not set or is invalid use a default throttle value of 10%
                    auto_ctrls_[i] = 26
                    rospy.logwarn("Marine drive parameter not set, marine drive set to default 10% throttle")
                # rospy.loginfo("Marine Throttle set to: %d", auto_ctrls_[i])
            elif i == 3:
                auto_ctrls_[i] = 127  # Keep land wheels centered
            elif i == 4:  # daq
                auto_ctrls_[i] = 1  # don't turn off the DAQ!
            elif i == 5:  # bilge pump
                auto_ctrls_[i] = 1  # turn on the bilge pump because I'll probably forget
            elif i == 7:
                auto_ctrls_[i] = 1  # keep the wheels up!
            elif i == 8:  # cooling pump
                auto_ctrls_[i] = 1  # turn on the cooling pump because I'll probably forget
            else:
                auto_ctrls_[i] = 0  # send centered or off signal
        auto_pub.publish(auto_ctrls_)
        autoCallback(auto_ctrls_)
        MET += 0.010  # 0.01 seconds per cycle since rate is 100Hz
        t += 0.010  # 0.01 increment on zigzag_rate timer t
        rospy.loginfo("MET: %f" % MET)
        met_pub.publish(MET)
        rate.sleep()  # must be at end of loop!
    if MET >= rospy.get_param("MET"):
        for i in range(len(auto_ctrls_)):
            if i == 0:
                auto_ctrls_[i] = 127 #center the nozzle
            elif i == 1:
                auto_ctrls_[i] = 127 #keep land steering centered
            elif i == 2:
                auto_ctrls_[i] = 0 #turn off the jet
                rospy.loginfo("Marine Throttle set to: %d", auto_ctrls_[i])
            elif i == 3:
                auto_ctrls_[i] = 127 #keep drive wheels undriven
            elif i == 4:  # daq
                auto_ctrls_[i] = 1  # don't turn off the DAQ!
            elif i == 5:  # bilge pump
                auto_ctrls_[i] = 1  # turn on the bilge pump because I'll probably forget
            elif i == 7:
                auto_ctrls_[i] = 1  # keep the wheels up!
            elif i == 8:  # cooling pump
                auto_ctrls_[i] = 1  # turn on the cooling pump because I'll probably forget
            else:
                auto_ctrls_[i] = 0  # send centered or off signal
        auto_pub.publish(auto_ctrls_)
        autoCallback(auto_ctrls_)
        rospy.logwarn("Automatic Maneuver Has Finished at MET: %f", MET)
        MET = 0  # reset the MET
        auto_reset = True
        go_mqs = False  # send a trigger to handshake to reset the automatic count
        go_pub.publish(go_mqs)


if __name__ == "__main__":
    rospy.init_node('mqs_maneuver', anonymous=True)
    go_mqs = False  # bool trigger
    auto_reset = False  # bool value for reseting auto_count in mqs_handshake
    MET = 0  # mission elapsed time in seconds
    init_ = True  # bool for setting initial loop
    while not rospy.is_shutdown():
        if init_:
            msg = 0
            prev_msg = msg
            init_ = False
        # subscribe to whole message, only look at start value
        rospy.Subscriber("cmd_ctrls", numpy_msg(cmd_ctrl), maneuver_node_start)
        rospy.spin()


        #condider writing the subscribe here and calling the maneuver node start, and passing cmd_ctrl through to trigger
