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
  :slots (_cmds ))

(defmethod xbee::cmd_ctrl
  (:init
   (&key
    ((:cmds __cmds) (make-array 16 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _cmds __cmds)
   self)
  (:cmds
   (&optional __cmds)
   (if __cmds (setq _cmds __cmds)) _cmds)
  (:serialization-length
   ()
   (+
    ;; uint8[16] _cmds
    (* 1    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[16] _cmds
     (princ _cmds s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[16] _cmds
   (setq _cmds (make-array 16 :element-type :char))
   (replace _cmds buf :start2 ptr-) (incf ptr- 16)
   ;;
   self)
  )

(setf (get xbee::cmd_ctrl :md5sum-) "65c47160f0c57a1a6dca334e2e8b2754")
(setf (get xbee::cmd_ctrl :datatype-) "xbee/cmd_ctrl")
(setf (get xbee::cmd_ctrl :definition-)
      "#Message where each sub command message for the MQS is published into

uint8[16] cmds

")



(provide :xbee/cmd_ctrl "65c47160f0c57a1a6dca334e2e8b2754")


