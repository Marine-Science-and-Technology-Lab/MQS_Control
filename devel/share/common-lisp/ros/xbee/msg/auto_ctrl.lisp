; Auto-generated. Do not edit!


(cl:in-package xbee-msg)


;//! \htmlinclude auto_ctrl.msg.html

(cl:defclass <auto_ctrl> (roslisp-msg-protocol:ros-message)
  ((auto_ctrls
    :reader auto_ctrls
    :initarg :auto_ctrls
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass auto_ctrl (<auto_ctrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <auto_ctrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'auto_ctrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbee-msg:<auto_ctrl> is deprecated: use xbee-msg:auto_ctrl instead.")))

(cl:ensure-generic-function 'auto_ctrls-val :lambda-list '(m))
(cl:defmethod auto_ctrls-val ((m <auto_ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbee-msg:auto_ctrls-val is deprecated.  Use xbee-msg:auto_ctrls instead.")
  (auto_ctrls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <auto_ctrl>) ostream)
  "Serializes a message object of type '<auto_ctrl>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'auto_ctrls))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <auto_ctrl>) istream)
  "Deserializes a message object of type '<auto_ctrl>"
  (cl:setf (cl:slot-value msg 'auto_ctrls) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'auto_ctrls)))
    (cl:dotimes (i 16)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<auto_ctrl>)))
  "Returns string type for a message object of type '<auto_ctrl>"
  "xbee/auto_ctrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'auto_ctrl)))
  "Returns string type for a message object of type 'auto_ctrl"
  "xbee/auto_ctrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<auto_ctrl>)))
  "Returns md5sum for a message object of type '<auto_ctrl>"
  "9d019e319a4cd95831863d035d7d77d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'auto_ctrl)))
  "Returns md5sum for a message object of type 'auto_ctrl"
  "9d019e319a4cd95831863d035d7d77d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<auto_ctrl>)))
  "Returns full string definition for message of type '<auto_ctrl>"
  (cl:format cl:nil "#Message where each sub command from simulink or trigger script for the MQS is published into~%~%uint8[16] auto_ctrls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'auto_ctrl)))
  "Returns full string definition for message of type 'auto_ctrl"
  (cl:format cl:nil "#Message where each sub command from simulink or trigger script for the MQS is published into~%~%uint8[16] auto_ctrls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <auto_ctrl>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'auto_ctrls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <auto_ctrl>))
  "Converts a ROS message object to a list"
  (cl:list 'auto_ctrl
    (cl:cons ':auto_ctrls (auto_ctrls msg))
))
