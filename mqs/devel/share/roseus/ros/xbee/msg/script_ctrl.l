;; Auto-generated. Do not edit!


(when (boundp 'xbee::script_ctrl)
  (if (not (find-package "XBEE"))
    (make-package "XBEE"))
  (shadow 'script_ctrl (find-package "XBEE")))
(unless (find-package "XBEE::SCRIPT_CTRL")
  (make-package "XBEE::SCRIPT_CTRL"))

(in-package "ROS")
;;//! \htmlinclude script_ctrl.msg.html


(defclass xbee::script_ctrl
  :super ros::object
  :slots (_script_ctrls ))

(defmethod xbee::script_ctrl
  (:init
   (&key
    ((:script_ctrls __script_ctrls) (make-array 5 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _script_ctrls __script_ctrls)
   self)
  (:script_ctrls
   (&optional __script_ctrls)
   (if __script_ctrls (setq _script_ctrls __script_ctrls)) _script_ctrls)
  (:serialization-length
   ()
   (+
    ;; uint8[5] _script_ctrls
    (* 1    5)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8[5] _script_ctrls
     (princ _script_ctrls s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8[5] _script_ctrls
   (setq _script_ctrls (make-array 5 :element-type :char))
   (replace _script_ctrls buf :start2 ptr-) (incf ptr- 5)
   ;;
   self)
  )

(setf (get xbee::script_ctrl :md5sum-) "f4ceca9c837a67ee537847b65bec8a91")
(setf (get xbee::script_ctrl :datatype-) "xbee/script_ctrl")
(setf (get xbee::script_ctrl :definition-)
      "#Message where each sub command from csv script is published into

uint8[5] script_ctrls

")



(provide :xbee/script_ctrl "f4ceca9c837a67ee537847b65bec8a91")


