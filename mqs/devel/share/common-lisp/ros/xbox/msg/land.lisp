; Auto-generated. Do not edit!


(cl:in-package xbox-msg)


;//! \htmlinclude land.msg.html

(cl:defclass <land> (roslisp-msg-protocol:ros-message)
  ((fwd
    :reader fwd
    :initarg :fwd
    :type cl:float
    :initform 0.0)
   (rev
    :reader rev
    :initarg :rev
    :type cl:float
    :initform 0.0)
   (strl
    :reader strl
    :initarg :strl
    :type cl:float
    :initform 0.0))
)

(cl:defclass land (<land>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <land>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'land)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbox-msg:<land> is deprecated: use xbox-msg:land instead.")))

(cl:ensure-generic-function 'fwd-val :lambda-list '(m))
(cl:defmethod fwd-val ((m <land>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:fwd-val is deprecated.  Use xbox-msg:fwd instead.")
  (fwd m))

(cl:ensure-generic-function 'rev-val :lambda-list '(m))
(cl:defmethod rev-val ((m <land>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:rev-val is deprecated.  Use xbox-msg:rev instead.")
  (rev m))

(cl:ensure-generic-function 'strl-val :lambda-list '(m))
(cl:defmethod strl-val ((m <land>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:strl-val is deprecated.  Use xbox-msg:strl instead.")
  (strl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <land>) ostream)
  "Serializes a message object of type '<land>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fwd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rev))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'strl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <land>) istream)
  "Deserializes a message object of type '<land>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fwd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rev) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'strl) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<land>)))
  "Returns string type for a message object of type '<land>"
  "xbox/land")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'land)))
  "Returns string type for a message object of type 'land"
  "xbox/land")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<land>)))
  "Returns md5sum for a message object of type '<land>"
  "edaf8b5219d5128d3faa0e4ddabb09c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'land)))
  "Returns md5sum for a message object of type 'land"
  "edaf8b5219d5128d3faa0e4ddabb09c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<land>)))
  "Returns full string definition for message of type '<land>"
  (cl:format cl:nil "# message file for land control commands~%~%float32 fwd~%float32 rev~%float32 strl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'land)))
  "Returns full string definition for message of type 'land"
  (cl:format cl:nil "# message file for land control commands~%~%float32 fwd~%float32 rev~%float32 strl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <land>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <land>))
  "Converts a ROS message object to a list"
  (cl:list 'land
    (cl:cons ':fwd (fwd msg))
    (cl:cons ':rev (rev msg))
    (cl:cons ':strl (strl msg))
))
