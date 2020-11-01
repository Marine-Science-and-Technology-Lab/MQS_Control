;; Auto-generated. Do not edit!


(when (boundp 'xbee::mqs_ctrl)
  (if (not (find-package "XBEE"))
    (make-package "XBEE"))
  (shadow 'mqs_ctrl (find-package "XBEE")))
(unless (find-package "XBEE::MQS_CTRL")
  (make-package "XBEE::MQS_CTRL"))

(in-package "ROS")
;;//! \htmlinclude mqs_ctrl.msg.html


(defclass xbee::mqs_ctrl
  :super ros::object
  :slots (_cmds ))

(defmethod xbee::mqs_ctrl
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

(setf (get xbee::mqs_ctrl :md5sum-) "65c47160f0c57a1a6dca334e2e8b2754")
(setf (get xbee::mqs_ctrl :datatype-) "xbee/mqs_ctrl")
(setf (get xbee::mqs_ctrl :definition-)
      "#Message where final commands for the MQS are concatenated to

uint8[16] cmds

")



(provide :xbee/mqs_ctrl "65c47160f0c57a1a6dca334e2e8b2754")


