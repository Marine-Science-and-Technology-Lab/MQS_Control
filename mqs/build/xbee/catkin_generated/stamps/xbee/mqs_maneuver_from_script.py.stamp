#!/usr/bin/env python
import rospy
import numpy as np
from xbee.msg import cmd_ctrl
from xbee.msg import auto_ctrl
from xbee.msg import script_ctrl
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Float32

readytopublish = False

global cmd_ctrls_latest
global script_ctrls_latest

cmd_ctrls_latest = None
script_ctrls_latest = bytearray()

# log auto message in the log
def autoCallback(auto_ctrls):
    rospy.loginfo("Tx: %s", auto_ctrls)

def scriptCallback(script_ctrls):
    rospy.loginfo("Script: %s", script_ctrls)
    global script_ctrls_latest
    # convert to bytearray
    script_ctrls_latest = bytearray(script_ctrls.script_ctrls)
    if script_ctrls_latest:
        global readytopublish
        readytopublish = True

# log the trigger incoming message in the log
def triggerCallback(cmd_ctrls):
    rospy.loginfo("Rx: %s", cmd_ctrls)
    global cmd_ctrls_latest
    cmd_ctrls_latest = cmd_ctrls


# looking for trigger from XBOX controller
def maneuver_trigger(cmd_ctrls,go_pub):
    global msg
    global prev_msg
    global go_mqs
    global auto_reset
    cmd_ctrls_ = bytearray()
    cmd_ctrls_.extend(cmd_ctrls.cmd_ctrls)
    #triggerCallback(cmd_ctrls) # see what the incoming message for debugging
    if cmd_ctrls_:  # if the message is full
        temp = str(cmd_ctrls_[11])  # convert int to string so we can print diagnostics
        msg = int(temp, 16)  # start button zero or one
        if msg == 1 and prev_msg == 0:
            rospy.loginfo("Trigger: " + temp)
            go_mqs = True
            rospy.loginfo("Go MQS!")
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



def maneuver_node_start(cmd_ctrls,auto_pub,met_pub,go_pub):
    global go_mqs
    global auto_reset
    # rospy.Subscriber("cmd_ctrls",numpy_msg(cmd_ctrl),triggerCallback)  #for debugging
    if not go_mqs and not rospy.is_shutdown():
        rospy.loginfo("Waiting for start from cmd_ctrl")
        maneuver_trigger(cmd_ctrls,go_pub)
    if go_mqs and not auto_reset and not rospy.is_shutdown() and readytopublish:
        # publish them controls
        rospy.loginfo("Automatic control Broadcasting...")
        mqs_maneuver_from_script(auto_pub,met_pub,go_pub)

def mqs_maneuver_from_script(auto_pub,met_pub,go_pub):

    global MET
    global auto_reset
    global go_mqs
    global script_ctrls_latest
    rospy.logwarn("Automatic Maneuver Control Enabled...")

    auto_ctrls = auto_ctrl()
    auto_ctrls_ = []
    auto_ctrls_.extend(auto_ctrls.auto_ctrls)

    # initialize the start time as the current ROS time
    start_time_MET = rospy.Time.now()

    # publish commands from the csv script
    while MET < rospy.get_param("MET") and not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time_MET = (current_time - start_time_MET).to_sec()
        script_ctrls_ = script_ctrls_latest
        for i in range(len(auto_ctrls_)):
            if i == 0:
                # STRM
                #rospy.loginfo("script_ctrls[0]: "+str(script_ctrls_[0]))
                auto_ctrls_[i] = script_ctrls_[0]
            elif i == 1:
                # LD
                #rospy.loginfo("script_ctrls[1]: " + str(script_ctrls_[1]))
                auto_ctrls_[i] = script_ctrls_[1]
            elif i == 2:
                # Waterjet
                #rospy.loginfo("script_ctrls[2]: " + str(script_ctrls_[2]))
                auto_ctrls_[i] = script_ctrls_[2]
            elif i == 3:
                # STRL
                #rospy.loginfo("script_ctrls[3]: " + str(script_ctrls_[3]))
                auto_ctrls_[i] = script_ctrls_[3]
            elif i == 5:
                # bilge pump on
                auto_ctrls_[i] = 1
            elif i == 7:
                # Wheel Retraction
                #rospy.loginfo("script_ctrls[4]: " + str(script_ctrls_[4]))
                auto_ctrls_[i] = script_ctrls_[4]
            elif i == 8:
                # Cooling pump on
                auto_ctrls_[i] = 1
            else:
                # ESC, DAQ, Reverse Marine, Abort, etc ... off
                auto_ctrls_[i] = 0
        autoCallback(auto_ctrls_)
        # publish the auto control message
        auto_pub.publish(auto_ctrls_)
        # update the MET
        MET = elapsed_time_MET
        # print and publish the MET
        rospy.loginfo("MET: %f" % MET)
        met_pub.publish(MET)
    # at conclusion of MET
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
                rospy.loginfo("Land drive set to: %d", auto_ctrls_[i])
            elif i == 4:  # daq
                auto_ctrls_[i] = 1  # don't turn off the DAQ!
            elif i == 5:  # bilge pump
                auto_ctrls_[i] = 1  # turn on the bilge pump because I'll probably forget
            elif i == 7:
                if rospy.get_param("auto_wheels_up_time") == -1:
                    auto_ctrls_[i] = 0  # don't raise the wheels
                else:
                    auto_ctrls_[i] = 1  # keep the wheels up!
            elif i == 8:  # cooling pump
                auto_ctrls_[i] = 1  # turn on the cooling pump because I'll probably forget
            else:
                auto_ctrls_[i] = 0  # send centered or off signal
        auto_pub.publish(auto_ctrls_)
        autoCallback(auto_ctrls_)
        rospy.logwarn("Automatic Maneuver Has Finished at MET: %f", MET)
        MET = 0  # reset the MET in the script
        auto_reset = True
        go_mqs = False  # send a trigger to handshake to reset the automatic count
        go_pub.publish(go_mqs)

if __name__ == "__main__":
    rospy.init_node('mqs_maneuver_from_script')
    go_mqs = False  # bool trigger
    auto_reset = False  # bool value for resetting auto_count
    MET = 0  # mission elapsed time in seconds
    init_ = True  # bool for setting initial loop

    # subscriber only needs to be set up once
    # subscribe to whole message, only look at start value
    rospy.Subscriber("cmd_ctrls", numpy_msg(cmd_ctrl),triggerCallback)
    rospy.Subscriber("script_ctrls",numpy_msg(script_ctrl),scriptCallback)

    auto_pub = rospy.Publisher('auto_ctrls', numpy_msg(auto_ctrl), queue_size=2)
    met_pub = rospy.Publisher('MET', Float32, queue_size=2)
    go_pub = rospy.Publisher("go_mqs", Bool, queue_size=1)
    rospy.sleep(1) # we need to let everything get started up

    # create rate outside of loop
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if init_:
            msg = 0
            prev_msg = msg
            init_ = False
        if cmd_ctrls_latest is not None:
            maneuver_node_start(cmd_ctrls_latest,auto_pub,met_pub,go_pub)
        rate.sleep()

