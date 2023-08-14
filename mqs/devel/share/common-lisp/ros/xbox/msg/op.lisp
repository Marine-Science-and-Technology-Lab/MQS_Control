; Auto-generated. Do not edit!


(cl:in-package xbox-msg)


;//! \htmlinclude op.msg.html

(cl:defclass <op> (roslisp-msg-protocol:ros-message)
  ((esc
    :reader esc
    :initarg :esc
    :type cl:fixnum
    :initform 0)
   (bp
    :reader bp
    :initarg :bp
    :type cl:fixnum
    :initform 0)
   (daq
    :reader daq
    :initarg :daq
    :type cl:fixnum
    :initform 0)
   (wrt
    :reader wrt
    :initarg :wrt
    :type cl:fixnum
    :initform 0)
   (cp
    :reader cp
    :initarg :cp
    :type cl:fixnum
    :initform 0)
   (rvm
    :reader rvm
    :initarg :rvm
    :type cl:fixnum
    :initform 0)
   (abort
    :reader abort
    :initarg :abort
    :type cl:fixnum
    :initform 0)
   (start
    :reader start
    :initarg :start
    :type cl:fixnum
    :initform 0))
)

(cl:defclass op (<op>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <op>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'op)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xbox-msg:<op> is deprecated: use xbox-msg:op instead.")))

(cl:ensure-generic-function 'esc-val :lambda-list '(m))
(cl:defmethod esc-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:esc-val is deprecated.  Use xbox-msg:esc instead.")
  (esc m))

(cl:ensure-generic-function 'bp-val :lambda-list '(m))
(cl:defmethod bp-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:bp-val is deprecated.  Use xbox-msg:bp instead.")
  (bp m))

(cl:ensure-generic-function 'daq-val :lambda-list '(m))
(cl:defmethod daq-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:daq-val is deprecated.  Use xbox-msg:daq instead.")
  (daq m))

(cl:ensure-generic-function 'wrt-val :lambda-list '(m))
(cl:defmethod wrt-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:wrt-val is deprecated.  Use xbox-msg:wrt instead.")
  (wrt m))

(cl:ensure-generic-function 'cp-val :lambda-list '(m))
(cl:defmethod cp-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:cp-val is deprecated.  Use xbox-msg:cp instead.")
  (cp m))

(cl:ensure-generic-function 'rvm-val :lambda-list '(m))
(cl:defmethod rvm-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:rvm-val is deprecated.  Use xbox-msg:rvm instead.")
  (rvm m))

(cl:ensure-generic-function 'abort-val :lambda-list '(m))
(cl:defmethod abort-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:abort-val is deprecated.  Use xbox-msg:abort instead.")
  (abort m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <op>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xbox-msg:start-val is deprecated.  Use xbox-msg:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <op>) ostream)
  "Serializes a message object of type '<op>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'esc)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'daq)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rvm)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abort)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <op>) istream)
  "Deserializes a message object of type '<op>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'esc)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'daq)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'wrt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rvm)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'abort)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'start)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<op>)))
  "Returns string type for a message object of type '<op>"
  "xbox/op")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'op)))
  "Returns string type for a message object of type 'op"
  "xbox/op")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<op>)))
  "Returns md5sum for a message object of type '<op>"
  "a88ed851050f435447fefa00180357ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'op)))
  "Returns md5sum for a message object of type 'op"
  "a88ed851050f435447fefa00180357ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<op>)))
  "Returns full string definition for message of type '<op>"
  (cl:format cl:nil "# Message files for all of the switch commands for the MQS~%~%uint8 esc  #turn on/off the esc's~%uint8 bp   #turn on/off the bilge pump~%uint8 daq  #turn on/off the DAQ~%uint8 wrt  #raise and lower the wheel retraction~%uint8 cp   #turn on/off the cooling pumps for the ESC's~%uint8 rvm  #hold to engage reverse mode for marine~%uint8 abort #abort joystick operation. Change over to transmitter on arduino~%uint8 start #start manuever switch to begin a keyed up manuever~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'op)))
  "Returns full string definition for message of type 'op"
  (cl:format cl:nil "# Message files for all of the switch commands for the MQS~%~%uint8 esc  #turn on/off the esc's~%uint8 bp   #turn on/off the bilge pump~%uint8 daq  #turn on/off the DAQ~%uint8 wrt  #raise and lower the wheel retraction~%uint8 cp   #turn on/off the cooling pumps for the ESC's~%uint8 rvm  #hold to engage reverse mode for marine~%uint8 abort #abort joystick operation. Change over to transmitter on arduino~%uint8 start #start manuever switch to begin a keyed up manuever~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <op>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <op>))
  "Converts a ROS message object to a list"
  (cl:list 'op
    (cl:cons ':esc (esc msg))
    (cl:cons ':bp (bp msg))
    (cl:cons ':daq (daq msg))
    (cl:cons ':wrt (wrt msg))
    (cl:cons ':cp (cp msg))
    (cl:cons ':rvm (rvm msg))
    (cl:cons ':abort (abort msg))
    (cl:cons ':start (start msg))
))
