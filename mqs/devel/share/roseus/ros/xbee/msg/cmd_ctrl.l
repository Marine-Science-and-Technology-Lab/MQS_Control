;; Auto-generated. Do not edit!


(when (boundp 'xbee::cmd_ctrl)
  (if (not (find-package "XBEE"))
    (make-package "XBEE"))
  (shadow 'cmd_ctrl (find-package "XBEE")))
(unless (find-package "XBEE::CMD_CTRL")
  (make-package "XBEE::CMD_CTRL"))

(in-package "ROS")
;;//! \htmlinclude cmd_ctrl.msg.html


(defclass xbee::cmd_ctrl
  :super ros::object
  :slots (_cmd_ctrls ))

(defmethod xbee::cmd_ctrl
  (:init
   (&key
    ((:cmd_ctrls __cmd_ctrls) (make-array 16 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _cmd_ctrls __cmd_ctrls)
   self)
  (:cmd_ctrls
   (&optional __cmd_ctrls)
   (if __cmd_ctrls (setq _cmd_ctrls __cmd_ctrls)) _cmd_ctrls)
  (:serialization-length
   ()
   (+
    ;; uint8[16] _cmd_ctrls
    (* 1    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[16] _cmd_ctrls
     (princ _cmd_ctrls s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[16] _cmd_ctrls
   (setq _cmd_ctrls (make-array 16 :element-type :char))
   (replace _cmd_ctrls buf :start2 ptr-) (incf ptr- 16)
   ;;
   self)
  )

(setf (get xbee::cmd_ctrl :md5sum-) "0180a78c48bb4a1e71184e050bf8b39c")
(setf (get xbee::cmd_ctrl :datatype-) "xbee/cmd_ctrl")
(setf (get xbee::cmd_ctrl :definition-)
      "#Message where each sub command from joy or keyboard message for the MQS is published into

uint8[16] cmd_ctrls

")



(provide :xbee/cmd_ctrl "0180a78c48bb4a1e71184e050bf8b39c")


