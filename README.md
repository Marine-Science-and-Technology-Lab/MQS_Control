# MQS_Control
MQS_Control is a ROS/Arduino control architecture designed for the IIHR Model Quad-Ski (MQS) robot. The goal of this project is to enhance the control functionality and introduce semi-autonomous capabilities to the MQS robot. The architecture has been structured to facilitate the integration of a closed-loop control program in the future, enabling full autonomous behavior both in the IIHR wave basin facility and in real-world scenarios.

This architecture was conceptualized by Mike Swafford as part of his Masters thesis in the Mechanical Engineering program at the University of Iowa.

## Arduino
The Arduino code is located in the `MQS_Robot_Control` folder. It includes both the foundational code and necessary header files for processing and queuing XBee messages. The base code features two control methods: ROS (default) and RC transmitter. The Arduino code reads data from either the XBee hardware serial (UART/RX 0) or the RC hardware serial (RX5), and translates these inputs into servo commands. While the code does not currently implement functionality to send messages back, the relevant sections for potential implementation are commented within the code.

## ROS
The ROS code is located in the `mqs` folder. To set up the ROS system, download the entire folder into a catkin workspace and use `catkin_make`. Currently, the following working packages are available: `mocap_base`, `mocap_qualysis`, `xbee`, and `xbox`. Note that other packages are still under development and not yet fully operational. For detailed information about each package, refer to the descriptions provided below.

### MOCAP_BASE
The `MOCAP_BASE` package serves as the central driver for the motion capture system. It forms the foundation of the `mocap_qualysis` node and incorporates essential components for its functioning.

- The `MOCAP_BASE` contains both the kalman filter implementation and the core driver. Header files related to this functionality can be found in the `include` folder, while the corresponding scripts are located within the `src` folder.
- This package's core purpose revolves around detecting the availability of information from Qualisys Track Manager (QTM), employing a kalman filter for data processing, and subsequently publishing raw messages containing the filtered data.

The functionality provided by `MOCAP_BASE` is designed to effectively interact with the Qualisys C++ SDK. While minimal changes are necessary for its core operations, you can explore this package further if you wish to integrate additional features from the Qualisys SDK.

### MOCAP_QUALYSIS
The `MOCAP_QUALYSIS` node serves as the core component for integrating the Qualisys motion capture system. Within the `include` folder, you'll find the complete SDK library. Key scripts are `qualysis` and `qualysisdriver`. The `qualysisdriver` is where the core functionality resides. It processes and publishes messages for both 6DOF and 3D marker data of the MQS robot.
- The 6DOF data, published as a "tf" (transform) under the `StampedTransform` ROS message, includes the transformation, frame ID, and a timestamp.
- The 3D marker data messages, named `marker_xyz`, are published under the `StampedVector3` ROS message. They encompass the X, Y, and Z coordinates of a marker, its name (e.g., "bow," "port"), and a timestamp.

This script also provides the flexibility to incorporate additional Qualisys SDK functionality such as triggering events and marking points. A basic launch file is available for debugging Qualisys data. The primary goal of this node is to supply position and pose data to SIMULINK for developing a closed-loop control algorithm for the MQS.

### XBEE
The `XBEE` node plays a crucial role in the management of the xbee radio transmitter, the implementation of the feed-forward control program, and the handling of message authority. Inside the `messages` folder, you will encounter three distinct custom message types:
- `auto_ctrl`: Automatic control messages intended for feed-forward programs or Simulink.
- `cmd_ctrl`: Modified joystick control messages that map Xbox controller inputs to specific robot functions.
- `mqs_ctrl`: Final messages that are transmitted via xbee after the `mqs_handshake` determines control authority.

Key aspects of this node include:
- `mqs_handshake`, located within the xbee source folder, decides which control message takes precedence. It bases this determination on the freshness of data published by nodes and the status of the mission elapsed time (MET) message from the feed-forward control scheme. If the MET is not active, joystick control prevails; if the MET is running, distinct controls are accessible to feed-forward and joystick controllers based on preset time parameters (specified in `mqs_teleop.launch` of the xbox node).
- Noteworthy exceptions to authority involve control timeouts (5 minutes for joystick, 1 second for feed-forward). Furthermore, a direct throttle override is implemented: any modification to the throttle during feed-forward operation halts feed-forward control and fully returns control to the joystick.

The `scripts` folder contains Python code files:
- `mqs_xbee` transmits control signals as byte objects over 802 RF to a robot receiver.
- `mqs_autoRelease` serves as the feed-forward control program that reacts to a trigger message to initiate mission elapsed time (MET), governing various driving and control operations.
- `mqs_maneuver.py` empowers pilots to execute predefined maneuvers such as straight-ahead, circle/donut, and zig-zag. Activation of these maneuvers involves specific parameter settings, enhancing control flexibility.

To activate the `mqs_manveuver` option, press the UP button on the Xbox D-pad.

### XBOX
The `XBOX` node forms the foundation of the MQS ROS packages. This node assumes the responsibility of managing the joy message and converting it into MQS controls using the Xbox controller.

- The core script, `mqs_teleop.py`, translates Xbox controller inputs into MQS controls, encompassing joystick inputs and Xbox controller buttons.
- The node also encompasses the primary launch file, `mqs_teleop.launch`, which resides in the `launch` folder. This launch file serves as the central configuration point for the project, initializing necessary nodes and parameters.

Key attributes of the `mqs_teleop.launch` include:
- The `joy` node, which establishes parameters for the raw joystick. This node requires two parameters: the USB port (default: `/dev/input/js0`) and the deadzone (recommended: 0.1 for smooth operation).
- The `mqs_teleop` node, which handles the conversion of Xbox controller parameters into MQS functions. Parameters within this node involve button layout configuration, scales for steering and throttles, and other dynamic settings.

Furthermore, the launch file initializes the `auto` node, setting parameters for the `mqs_autoRelease` feed-forward control. Parameters govern various aspects, such as land drive speed, mission elapsed time (MET) duration, marine throttle, wheels-up timing, and steering angles.

The `Qualysis` node sets parameters for the Qualisys Track Manager (QTM) server, including the server address, base port, frame ID, UDP settings, and model and marker lists.

This comprehensive node facilitates dynamic interaction and control of the MQS robot using the Xbox controller, making it a pivotal component of the project.


### Other important notes
Before turning on the teensy on boot up of "roslaunch xbox mqs_teleop.launch" the two xbox controller triggers (LT and RT) need to be depressed and released, this calibrates
the controller, it must be done each time the nodes are started! You can turn on the teensy after this has been completed.

If the mqs_autoRelease receives a nonetype on the xbee it will kill the node, this is an issue being worked on, but in the meantime the easiest fix is to reset and reboot 

<figure align="center">
  <figcaption>Credit: Julian Alvarado and Nate Bastianen</figcaption>
  <img src="https://github.com/Swaffles/MQS_Control/blob/b63a98e6f8f0c61810bd075abcfd8ddce449a0bd/drawing%20finished.svg" alt="MQS XBOX Controller Layout">
</figure>
<figure align="center">
  <img src="https://github.com/Swaffles/MQS_Control/assets/58667766/d8164099-71f5-485f-9658-de0143ebf149" alt = "MQS R/C Joystick Layout">
</figure>  

# Contributing
Contributions to this project are welcome. If you have ideas for improvements, additional functionalities, or bug fixes, feel free to submit pull requests or open issues.

# License
This project is licensed under the [Creative Commons Zero v1.0 Universal](https://github.com/Swaffles/MQS_Control/blob/master/LICENSE).


