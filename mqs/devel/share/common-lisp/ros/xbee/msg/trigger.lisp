; Auto-generated. Do not edit!


(cl:in-package xbee-msg)


;//! \htmlinclude trigger.msg.html

(cl:defclass <trigger> (roslisp-msg-protocol:ros-message)
  ((go_mqs
    :reader go_mqs
    :initarg :go_mqs
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass trigger (<trigger>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trigger>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trigger)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbee-msg:<trigger> is deprecated: use xbee-msg:trigger instead.")))

(cl:ensure-generic-function 'go_mqs-val :lambda-list '(m))
(cl:defmethod go_mqs-val ((m <trigger>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbee-msg:go_mqs-val is deprecated.  Use xbee-msg:go_mqs instead.")
  (go_mqs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trigger>) ostream)
  "Serializes a message object of type '<trigger>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'go_mqs) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trigger>) istream)
  "Deserializes a message object of type '<trigger>"
    (cl:setf (cl:slot-value msg 'go_mqs) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trigger>)))
  "Returns string type for a message object of type '<trigger>"
  "xbee/trigger")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trigger)))
  "Returns string type for a message object of type 'trigger"
  "xbee/trigger")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trigger>)))
  "Returns md5sum for a message object of type '<trigger>"
  "a233c8f8bcc02b3d95334687e9718eb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trigger)))
  "Returns md5sum for a message object of type 'trigger"
  "a233c8f8bcc02b3d95334687e9718eb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trigger>)))
  "Returns full string definition for message of type '<trigger>"
  (cl:format cl:nil "#Trigger message to tell mqs_handshake that auto_ctrls is available~%bool go_mqs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trigger)))
  "Returns full string definition for message of type 'trigger"
  (cl:format cl:nil "#Trigger message to tell mqs_handshake that auto_ctrls is available~%bool go_mqs~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trigger>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trigger>))
  "Converts a ROS message object to a list"
  (cl:list 'trigger
    (cl:cons ':go_mqs (go_mqs msg))
))
