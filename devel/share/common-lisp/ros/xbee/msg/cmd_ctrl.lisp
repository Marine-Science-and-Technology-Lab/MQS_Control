; Auto-generated. Do not edit!


(cl:in-package xbee-msg)


;//! \htmlinclude cmd_ctrl.msg.html

(cl:defclass <cmd_ctrl> (roslisp-msg-protocol:ros-message)
  ((cmds
    :reader cmds
    :initarg :cmds
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass cmd_ctrl (<cmd_ctrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_ctrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_ctrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbee-msg:<cmd_ctrl> is deprecated: use xbee-msg:cmd_ctrl instead.")))

(cl:ensure-generic-function 'cmds-val :lambda-list '(m))
(cl:defmethod cmds-val ((m <cmd_ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbee-msg:cmds-val is deprecated.  Use xbee-msg:cmds instead.")
  (cmds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_ctrl>) ostream)
  "Serializes a message object of type '<cmd_ctrl>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'cmds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_ctrl>) istream)
  "Deserializes a message object of type '<cmd_ctrl>"
  (cl:setf (cl:slot-value msg 'cmds) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'cmds)))
    (cl:dotimes (i 16)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_ctrl>)))
  "Returns string type for a message object of type '<cmd_ctrl>"
  "xbee/cmd_ctrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_ctrl)))
  "Returns string type for a message object of type 'cmd_ctrl"
  "xbee/cmd_ctrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_ctrl>)))
  "Returns md5sum for a message object of type '<cmd_ctrl>"
  "65c47160f0c57a1a6dca334e2e8b2754")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_ctrl)))
  "Returns md5sum for a message object of type 'cmd_ctrl"
  "65c47160f0c57a1a6dca334e2e8b2754")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_ctrl>)))
  "Returns full string definition for message of type '<cmd_ctrl>"
  (cl:format cl:nil "#Message where each sub command message for the MQS is published into~%~%uint8[16] cmds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_ctrl)))
  "Returns full string definition for message of type 'cmd_ctrl"
  (cl:format cl:nil "#Message where each sub command message for the MQS is published into~%~%uint8[16] cmds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_ctrl>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'cmds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_ctrl>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_ctrl
    (cl:cons ':cmds (cmds msg))
))
