; Auto-generated. Do not edit!


(cl:in-package xbox-msg)


;//! \htmlinclude marine.msg.html

(cl:defclass <marine> (roslisp-msg-protocol:ros-message)
  ((thl
    :reader thl
    :initarg :thl
    :type cl:float
    :initform 0.0)
   (strm
    :reader strm
    :initarg :strm
    :type cl:float
    :initform 0.0))
)

(cl:defclass marine (<marine>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <marine>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'marine)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbox-msg:<marine> is deprecated: use xbox-msg:marine instead.")))

(cl:ensure-generic-function 'thl-val :lambda-list '(m))
(cl:defmethod thl-val ((m <marine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:thl-val is deprecated.  Use xbox-msg:thl instead.")
  (thl m))

(cl:ensure-generic-function 'strm-val :lambda-list '(m))
(cl:defmethod strm-val ((m <marine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:strm-val is deprecated.  Use xbox-msg:strm instead.")
  (strm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <marine>) ostream)
  "Serializes a message object of type '<marine>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'strm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <marine>) istream)
  "Deserializes a message object of type '<marine>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strm) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<marine>)))
  "Returns string type for a message object of type '<marine>"
  "xbox/marine")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'marine)))
  "Returns string type for a message object of type 'marine"
  "xbox/marine")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<marine>)))
  "Returns md5sum for a message object of type '<marine>"
  "e8ddfec22a5ef4b813c4ad7dee7890ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'marine)))
  "Returns md5sum for a message object of type 'marine"
  "e8ddfec22a5ef4b813c4ad7dee7890ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<marine>)))
  "Returns full string definition for message of type '<marine>"
  (cl:format cl:nil "# Message file for marine commands~%~%float32 thl	#controls the throttle~%float32 strm	#controls the waterjet nozzle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'marine)))
  "Returns full string definition for message of type 'marine"
  (cl:format cl:nil "# Message file for marine commands~%~%float32 thl	#controls the throttle~%float32 strm	#controls the waterjet nozzle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <marine>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <marine>))
  "Converts a ROS message object to a list"
  (cl:list 'marine
    (cl:cons ':thl (thl msg))
    (cl:cons ':strm (strm msg))
))
