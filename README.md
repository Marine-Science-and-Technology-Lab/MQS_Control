# MQS_Control
This is a ROS/Arduino control architecture built for the purpose of controlling the IIHR Model Quad-Ski (MQS) robot. The purpose of this code was to add more control functionality,
as well as some semi-autonomous function to the MQS robot. The architecture is setup so that future work on a closed-loop control program can be implemented in the future to 
add full autonomous behavior both in the IIHR wave basin facitlity and out in the field. This architecure was designed by Mike Swafford for the partial fullfilment of a 
Masters in mechanical engineering from the University of Iowa.

# Arduino
The Arduino code is listed under the MQS_Robot_Control folder.
It contains the base code as well as needed header files for interpriting and queuing xbee messages. The base code has two control methods, ROS (default) and RC transmitter.
The arduino code only reads data from either the Xbee hardware series (UART/ RX 0) and executes servo commands. No functionality to send back messages is currently implimented,
however, the location where it could be done is commented in the code.

# ROS
The ROS code is listed in the mqs folder (built, devel, and src are holdovers that did not get properly deleted in the last push, they will be in future updates).
Download the entire folder in a catkin_workspace and catkin_make to set up the ROS system. Current working packages are: mocap_base, mocap_qualysis, xbee, and xbox. All other
packages are not fully built. See description of each package below

# MOCAP_BASE
This is the main driver for the motion capture system. It contains the basic workings for the mocap_qualysis node. It contains a kalman filter as well as the base driver, 
the header files for which are located in the include folder and the scripts located in the src folder. The basic function is to detect whether information from QTM is available,
filter data, and publish raw messages about the data. No editing needs to be done here unless you wish to emply more features from the qualisys C++ SDK.

# MOCAP_QUALYSIS
This is the main node for the qualysis motion capture system. It contains the entire SDK library in the include folder. Main scripts are qualysis and qualysisdriver. The driver
is where all the functionality ocurrs. It processes and publishes messages for both the 6DOF and 3D marker data for the MQS robot. The 6DOF data published is called tf and
published under the StampedTransform ROS message it contains the transformation, frame_id, and a time stamp. 3D marker messages are called marker_xyz and published under the 
StampedVector3 ROS message, they contain the X, Y, and Z of a marker, the markers name (bow, port, etc.) and a time stamp. This script is also where you can add more
qualysis SDK functionality like triggering, marking events, etc. A simple launch file is also available in this package, it's purpose is for debugging QTM data. The nodes purpose
is to provide position and pose data to SIMULINK for the purpose of developing a closed-loop control algorithm for the MQS.

# XBEE
This is the main node for operating the xbee radio transmitter, the feed-forward control program, as well as handling message authority. In the messages folder there are three 
custom message types: auto_ctrl is the auto control message type that either the feed-forward control program or simulink can publish to. cmd_ctrl is the modified joystick 
control message where xbox control inputs are mapped to specific robot functions. mqs_ctrl is the final published message that is sent over the xbee after mqs_handshake decides
which control type has authority, joystick or feed-forward. mqs_handshake is located int he xbee src folder. As stated before mqs_handshake decides which message has priority
to control the robot. It makes this decision based on which node is publishing fresh data and wheter the mission elapsed time (MET) message from the feed-forward control scheme
is running. If the MET is not running handshake activateds joystick override passing all control to the xbox controller. If the MET is running, certain controls will be available
to the feed-forward scheme and others available to the xbox controller, these are mainly determined by time parameters set on the main parameter server (See mqs_teleop.launch in
the xbox node for more details). In the scripts folder is where the python codes are located, mqs_xbee is only used to send the control signals as byte objects over 802 rf to 
a reciever on the robot. mqs_autoRelease is the feed-forward control program that handels recieving a trigger message from another computer (for our purpose this was a comuter 
monitoriing wave phase in labVIEW) the trigger message is simply a 01 in hex with all the appropriate xbee header information. Once the start bit is recieved the mission elapsed
time clock starts (MET) the time is based on the parameter set on the parameter server (see xbox mqs_teleop.launch). During the MET several different driving and control
operations can be performed. By default it is set for outbound cases at a crawl speed for 4 seconds, a marine throttle of 10%, and wheels up after 2 seconds. All feed-forward parameters can be set on the
parameter server before or during operation. For inbound cases the signs on the land drive section (i=1) must be swapped (see comments in code) this will change the drive time
to a delta meaning they will start driving after a specified time. For inbound the wheels up time also should be changed to 0 and should be set on the joystick before launching
to prevent the wheels from locking in the up position. It should be noted that for ease of operation once the MET has finished all control will be given back to the driver, however,
the auto throttle will remain on until the pilot intervenes, this acts as a sort of cruise control that makes drivning the model in the water a single stick operation. To remove 
this feature simply add:
 && mqs_met < MET_END 
 to the if statment for i=2 in the mqs_handshake autoCallback function

# XBOX
This is the meat and potatos of the MQS ROS packages. This node handles the joy message and converts to MQS controls on the xbox controller. This node also houses the main launch
file in the launch folder called mqs_teleop.launch. All buttons and axis on the joystick are configurable, however, we found the defualts to be the easiest to drive in such a
dynamic environment as waves. The mqs_teleop also scales all the joystick outputs to the correct byte types, sets the deadzones and determines what values are valid. The launch
file mqs_teleop.launch found in the xbox launch folder is the main ROS launch file for the project. Running roslaunch xbox mqs_teleop.launch starts up all the necessary nodes
and initializes the parameter server to the default value. Important items in the launch file are as follows:

Joy node: sets parameters for the raw joystick, has two parameters, the first is the usb port, defualt is /dev/input/js0. The second parameter is the deadzone, for smooth operation
we found 0.1 to be best.

mqs teleop node: sets the conversion of xbox controller parameters to MQS functions. The first set of parameters deal with button layout, these maybe changed as desired, but must
be set before launching. The scales parameters set the range of the steering, and throttles. The only one that you can change on the fly is scale_throttle which will set the max
marine throttle to what ever value you desire. It is recommened at this time to not exceed 165.

auto node: sets auto parameters for the mqs_autoRelease feed-forward control. All the parameters are labeled but a quick summary is available here:
auto_land_drive sets the land drive speed, you should only be entering values greater than 127. Defualt is 0 which is hardcoded to 160 (crawl) in the autoRelease function.
auto_land_drive_time sets the duration for the land time to run. defualt is 4 seconds
MET sets the durations for the mission elapsed time clock. defualt is 10 seconds
auto_marine_drive sets the marine throttle. defualt is 0 which is hardcoded to 26 (10%)
auto_wheels_up_time sets the time when the wheels will retract. defualt is 2 seconds (transiion time is about 2-3 seconds)
auto_land_steer sets the land steering angle. towards 255 goes left, towards 0 goes right. default is 127 for straight ahead.
auto_land_steer_time sets the time for the land steering. defualt is 0. Generallhy when in use I set this the same as the auto wheels up time
auto_marine_steer sets the steering angle for the waterjet. towards 255 goes left, towards 0 goes right. defualt is 127 for straight ahead.
auto_marine_steer_time sets the time for auto marine steer. defualt is 0. Generally I set this pretty high for small angles (near 127) and low for larger angles

Qualysis node: sets the parameters for the QTM server
server address is the IPV4 address of the machine running qualysis
server base port is the port selecting in setting for real time QTM. default is 22222
fixed frame id sets the name for tf message name
udp_port sets the udp. -1 sets to TCP/IP and is the default
model list and marker list are dynamically allocated list for tracking objects or markers. you do not need to set these if your QTM data has labels


# Other important notes
Before turning on the teensy on boot up of "roslaunch xbox mqs_teleop.launch" the two xbox controller triggers (LT and RT) need to be depressed and released, this calibrates
the controller, it must be done each time the nodes are started! You can turn on the teensy after this has been completed.

If the mqs_autoRelease receives a nonetype on the xbee it will kill the node, this is an issue being worked on, but in the meantime the easiest fix is to reset and reboot 

