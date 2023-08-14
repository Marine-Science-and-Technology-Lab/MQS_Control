;; Auto-generated. Do not edit!


(when (boundp 'xbox::op)
  (if (not (find-package "XBOX"))
    (make-package "XBOX"))
  (shadow 'op (find-package "XBOX")))
(unless (find-package "XBOX::OP")
  (make-package "XBOX::OP"))

(in-package "ROS")
;;//! \htmlinclude op.msg.html


(defclass xbox::op
  :super ros::object
  :slots (_esc _bp _daq _wrt _cp _rvm _abort _start ))

(defmethod xbox::op
  (:init
   (&key
    ((:esc __esc) 0)
    ((:bp __bp) 0)
    ((:daq __daq) 0)
    ((:wrt __wrt) 0)
    ((:cp __cp) 0)
    ((:rvm __rvm) 0)
    ((:abort __abort) 0)
    ((:start __start) 0)
    )
   (send-super :init)
   (setq _esc (round __esc))
   (setq _bp (round __bp))
   (setq _daq (round __daq))
   (setq _wrt (round __wrt))
   (setq _cp (round __cp))
   (setq _rvm (round __rvm))
   (setq _abort (round __abort))
   (setq _start (round __start))
   self)
  (:esc
   (&optional __esc)
   (if __esc (setq _esc __esc)) _esc)
  (:bp
   (&optional __bp)
   (if __bp (setq _bp __bp)) _bp)
  (:daq
   (&optional __daq)
   (if __daq (setq _daq __daq)) _daq)
  (:wrt
   (&optional __wrt)
   (if __wrt (setq _wrt __wrt)) _wrt)
  (:cp
   (&optional __cp)
   (if __cp (setq _cp __cp)) _cp)
  (:rvm
   (&optional __rvm)
   (if __rvm (setq _rvm __rvm)) _rvm)
  (:abort
   (&optional __abort)
   (if __abort (setq _abort __abort)) _abort)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; uint8 _esc
    1
    ;; uint8 _bp
    1
    ;; uint8 _daq
    1
    ;; uint8 _wrt
    1
    ;; uint8 _cp
    1
    ;; uint8 _rvm
    1
    ;; uint8 _abort
    1
    ;; uint8 _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _esc
       (write-byte _esc s)
     ;; uint8 _bp
       (write-byte _bp s)
     ;; uint8 _daq
       (write-byte _daq s)
     ;; uint8 _wrt
       (write-byte _wrt s)
     ;; uint8 _cp
       (write-byte _cp s)
     ;; uint8 _rvm
       (write-byte _rvm s)
     ;; uint8 _abort
       (write-byte _abort s)
     ;; uint8 _start
       (write-byte _start s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _esc
     (setq _esc (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _bp
     (setq _bp (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _daq
     (setq _daq (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _wrt
     (setq _wrt (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _cp
     (setq _cp (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _rvm
     (setq _rvm (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _abort
     (setq _abort (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _start
     (setq _start (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get xbox::op :md5sum-) "a88ed851050f435447fefa00180357ec")
(setf (get xbox::op :datatype-) "xbox/op")
(setf (get xbox::op :definition-)
      "# Message files for all of the switch commands for the MQS

uint8 esc  #turn on/off the esc's
uint8 bp   #turn on/off the bilge pump
uint8 daq  #turn on/off the DAQ
uint8 wrt  #raise and lower the wheel retraction
uint8 cp   #turn on/off the cooling pumps for the ESC's
uint8 rvm  #hold to engage reverse mode for marine
uint8 abort #abort joystick operation. Change over to transmitter on arduino
uint8 start #start manuever switch to begin a keyed up manuever

")



(provide :xbox/op "a88ed851050f435447fefa00180357ec")


