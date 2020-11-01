;; Auto-generated. Do not edit!


(when (boundp 'xbee::auto_ctrl)
  (if (not (find-package "XBEE"))
    (make-package "XBEE"))
  (shadow 'auto_ctrl (find-package "XBEE")))
(unless (find-package "XBEE::AUTO_CTRL")
  (make-package "XBEE::AUTO_CTRL"))

(in-package "ROS")
;;//! \htmlinclude auto_ctrl.msg.html


(defclass xbee::auto_ctrl
  :super ros::object
  :slots (_auto_ctrls ))

(defmethod xbee::auto_ctrl
  (:init
   (&key
    ((:auto_ctrls __auto_ctrls) (make-array 16 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _auto_ctrls __auto_ctrls)
   self)
  (:auto_ctrls
   (&optional __auto_ctrls)
   (if __auto_ctrls (setq _auto_ctrls __auto_ctrls)) _auto_ctrls)
  (:serialization-length
   ()
   (+
    ;; uint8[16] _auto_ctrls
    (* 1    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[16] _auto_ctrls
     (princ _auto_ctrls s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[16] _auto_ctrls
   (setq _auto_ctrls (make-array 16 :element-type :char))
   (replace _auto_ctrls buf :start2 ptr-) (incf ptr- 16)
   ;;
   self)
  )

(setf (get xbee::auto_ctrl :md5sum-) "9d019e319a4cd95831863d035d7d77d8")
(setf (get xbee::auto_ctrl :datatype-) "xbee/auto_ctrl")
(setf (get xbee::auto_ctrl :definition-)
      "#Message where each sub command from simulink or mqs_autoRelease for the MQS is published into

uint8[16] auto_ctrls

")



(provide :xbee/auto_ctrl "9d019e319a4cd95831863d035d7d77d8")


