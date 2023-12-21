#!/usr/bin/env python2
import rospy
import numpy as np
import pandas as pd
import os
from xbee.msg import script_ctrl
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool

go_mqs_value = False

def go_mqs_trigger(msg):
    global go_mqs_value
    go_mqs_value = msg.data
    if go_mqs_value:
        rospy.loginfo("Trigger Received, publish script active")


def scriptCallback(msg):
    rospy.loginfo("RX: %s", msg)


def read_file(csv_file_path, last_mod_time, prev_df):
    current_mod_time = os.path.getmtime(csv_file_path)
    if current_mod_time != last_mod_time:
        try:
            df = pd.read_csv(csv_file_path)
        except Commas:
            df = pd.read_csv(csv_file_path, sep='\t')

        # check the column names match
        #rospy.loginfo(df.columns)
        rospy.loginfo(df.head())

        # Return the current_mod_time to update last_mod_time in main and flag modification as true
        return df, current_mod_time, True
    else:
        # return the prev_df
        return prev_df, last_mod_time, False

def publish_script(df,script_pub,write_rate):
    # if all the parameters are valid they are written to the server, otherwise validate_all_params raises errors
    # on the master node
    rospy.loginfo("Script is Valid")
    # get the last value in the 'time' column
    last_time_value = df['t'].iloc[-1]
    # ensure the correct python type for ROS
    last_time_value = float(last_time_value)
    # set the MET
    rospy.set_param("MET", last_time_value)
    rospy.logwarn("MET set to: "+str(last_time_value)+" by script")
    # set STRM time to MET
    rospy.set_param("auto_marine_steer_time",last_time_value)
    # set STRL time to MET
    rospy.set_param("auto_land_steer_time",last_time_value)
    # set LD time to MET
    rospy.set_param("auto_land_drive_time",last_time_value)
    for index, row in df.iterrows():
        if rospy.is_shutdown():
            break

        # print the time
        rospy.loginfo("Time from Script: "+str(row['t']))
        row_data = np.array(row.drop('t'), dtype=np.uint8)
        # Create and publish the msg
        msg = script_ctrl()
        msg.script_ctrls = row_data.tolist()
        scriptCallback(msg)
        script_pub.publish(msg)

        write_rate.sleep()

def validate_script(df,column_ranges):
    for column_name, (expected_type, (min_val, max_val)) in column_ranges.items():
        # convert te column to the expected type
        # print the column type
        #rospy.loginfo("Name: "+column_name)
        #rospy.loginfo("Expected type: "+str(expected_type))
        #rospy.loginfo("Range: ["+str(min_val)+","+str(max_val)+"]")
        #rospy.loginfo(df[column_name].head())
        #rospy.loginfo("Last value in column: "+str(df[column_name].iloc[-1]))

        if column_name not in df.columns:
            rospy.loginfo("Column: "+column_name+" not found in DataFrame")
            return False

        try:
            if expected_type is float:
                df[column_name] = df[column_name].astype(float)
            elif expected_type is int:
                df[column_name] = df[column_name].astype(int)
        except ValueError:
            rospy.logwarn("Conversion to "+expected_type+" for "+column_name+" failed")
            return False

        # check for NaN values which indicate failed conversions
        if df[column_name].isna().any():
            rospy.logwarn("NaN values found in column "+column_name+", indicating failed conversion")
            return False

        # check the range
        if not ((df[column_name] >= min_val) & (df[column_name] <= max_val)).all():
            rospy.logwarn("Value in column "+column_name+" is out of range ["+str(min_val)+","+str(max_val)+"]")
            return False
    return df, True




def main():
    rospy.init_node('mqs_rw_csv_script')
    #rospy.loginfo("Node init success")
    csv_file_path = rospy.get_param('script_path', '/home/lab/Desktop/ROS_Scripts/Test01.csv')
    #rospy.loginfo("csv_file_path set")
    last_mod_time = 0
    read_rate = rospy.Rate(1) # check for modifications to the file once every second (1 Hz)
    write_rate = rospy.Rate(100) # write script to cmd_ctrl at 100 Hz
    script_pub = rospy.Publisher('script_ctrls', numpy_msg(script_ctrl), queue_size = 10)
    #rospy.loginfo("script_ctrls publisher setup success")
    rospy.Subscriber('go_mqs',Bool,go_mqs_trigger)
    #rospy.loginfo("Subscribed to go_mqs")

    # Expected values for each control signal
    column_ranges = {
        "t": (float, (0.0, 60.0)),  # no scripts longer than 60 seconds
        "strm": (int, (0, 255)),
        "ld": (int, (0, 255)),
        "jet": (int, (0, 255)),
        "strl": (int, (0, 255)),
        "wr": (int, (0, 1)),
    }

    # validate script init param
    valid = False
    # stores the prev_df, so that read_file doesn't output None after the first time it reads the file
    prev_df = None
    while not rospy.is_shutdown():
        df, last_mod_time, file_updated, = read_file(csv_file_path,last_mod_time, prev_df)
        if file_updated:
            # validate all the parameters
            df, valid = validate_script(df, column_ranges)
            prev_df = df
        if go_mqs_value and valid:
            publish_script(df, script_pub, write_rate)
        read_rate.sleep()

if __name__ == '__main__':
    main()