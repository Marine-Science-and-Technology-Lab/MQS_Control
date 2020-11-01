;; Auto-generated. Do not edit!


(when (boundp 'xbee::trigger)
  (if (not (find-package "XBEE"))
    (make-package "XBEE"))
  (shadow 'trigger (find-package "XBEE")))
(unless (find-package "XBEE::TRIGGER")
  (make-package "XBEE::TRIGGER"))

(in-package "ROS")
;;//! \htmlinclude trigger.msg.html


(defclass xbee::trigger
  :super ros::object
  :slots (_go_mqs ))

(defmethod xbee::trigger
  (:init
   (&key
    ((:go_mqs __go_mqs) nil)
    )
   (send-super :init)
   (setq _go_mqs __go_mqs)
   self)
  (:go_mqs
   (&optional __go_mqs)
   (if __go_mqs (setq _go_mqs __go_mqs)) _go_mqs)
  (:serialization-length
   ()
   (+
    ;; bool _go_mqs
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _go_mqs
       (if _go_mqs (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _go_mqs
     (setq _go_mqs (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get xbee::trigger :md5sum-) "a233c8f8bcc02b3d95334687e9718eb4")
(setf (get xbee::trigger :datatype-) "xbee/trigger")
(setf (get xbee::trigger :definition-)
      "#Trigger message to tell mqs_handshake that auto_ctrls is available
bool go_mqs

")



(provide :xbee/trigger "a233c8f8bcc02b3d95334687e9718eb4")


