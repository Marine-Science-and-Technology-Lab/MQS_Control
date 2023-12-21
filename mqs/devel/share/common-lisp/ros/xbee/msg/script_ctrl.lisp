; Auto-generated. Do not edit!


(cl:in-package xbee-msg)


;//! \htmlinclude script_ctrl.msg.html

(cl:defclass <script_ctrl> (roslisp-msg-protocol:ros-message)
  ((script_ctrls
    :reader script_ctrls
    :initarg :script_ctrls
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 5 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass script_ctrl (<script_ctrl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <script_ctrl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'script_ctrl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbee-msg:<script_ctrl> is deprecated: use xbee-msg:script_ctrl instead.")))

(cl:ensure-generic-function 'script_ctrls-val :lambda-list '(m))
(cl:defmethod script_ctrls-val ((m <script_ctrl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbee-msg:script_ctrls-val is deprecated.  Use xbee-msg:script_ctrls instead.")
  (script_ctrls m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <script_ctrl>) ostream)
  "Serializes a message object of type '<script_ctrl>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'script_ctrls))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <script_ctrl>) istream)
  "Deserializes a message object of type '<script_ctrl>"
  (cl:setf (cl:slot-value msg 'script_ctrls) (cl:make-array 5))
  (cl:let ((vals (cl:slot-value msg 'script_ctrls)))
    (cl:dotimes (i 5)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<script_ctrl>)))
  "Returns string type for a message object of type '<script_ctrl>"
  "xbee/script_ctrl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'script_ctrl)))
  "Returns string type for a message object of type 'script_ctrl"
  "xbee/script_ctrl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<script_ctrl>)))
  "Returns md5sum for a message object of type '<script_ctrl>"
  "f4ceca9c837a67ee537847b65bec8a91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'script_ctrl)))
  "Returns md5sum for a message object of type 'script_ctrl"
  "f4ceca9c837a67ee537847b65bec8a91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<script_ctrl>)))
  "Returns full string definition for message of type '<script_ctrl>"
  (cl:format cl:nil "#Message where each sub command from csv script is published into~%~%uint8[5] script_ctrls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'script_ctrl)))
  "Returns full string definition for message of type 'script_ctrl"
  (cl:format cl:nil "#Message where each sub command from csv script is published into~%~%uint8[5] script_ctrls~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <script_ctrl>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'script_ctrls) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <script_ctrl>))
  "Converts a ROS message object to a list"
  (cl:list 'script_ctrl
    (cl:cons ':script_ctrls (script_ctrls msg))
))
