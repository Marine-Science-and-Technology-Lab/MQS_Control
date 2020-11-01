;; Auto-generated. Do not edit!


(when (boundp 'xbox::marine)
  (if (not (find-package "XBOX"))
    (make-package "XBOX"))
  (shadow 'marine (find-package "XBOX")))
(unless (find-package "XBOX::MARINE")
  (make-package "XBOX::MARINE"))

(in-package "ROS")
;;//! \htmlinclude marine.msg.html


(defclass xbox::marine
  :super ros::object
  :slots (_thl _strm ))

(defmethod xbox::marine
  (:init
   (&key
    ((:thl __thl) 0.0)
    ((:strm __strm) 0.0)
    )
   (send-super :init)
   (setq _thl (float __thl))
   (setq _strm (float __strm))
   self)
  (:thl
   (&optional __thl)
   (if __thl (setq _thl __thl)) _thl)
  (:strm
   (&optional __strm)
   (if __strm (setq _strm __strm)) _strm)
  (:serialization-length
   ()
   (+
    ;; float32 _thl
    4
    ;; float32 _strm
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _thl
       (sys::poke _thl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _strm
       (sys::poke _strm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _thl
     (setq _thl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _strm
     (setq _strm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xbox::marine :md5sum-) "e8ddfec22a5ef4b813c4ad7dee7890ed")
(setf (get xbox::marine :datatype-) "xbox/marine")
(setf (get xbox::marine :definition-)
      "# Message file for marine commands

float32 thl	#controls the throttle
float32 strm	#controls the waterjet nozzle

")



(provide :xbox/marine "e8ddfec22a5ef4b813c4ad7dee7890ed")


