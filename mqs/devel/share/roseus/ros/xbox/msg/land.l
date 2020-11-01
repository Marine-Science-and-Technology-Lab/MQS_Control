;; Auto-generated. Do not edit!


(when (boundp 'xbox::land)
  (if (not (find-package "XBOX"))
    (make-package "XBOX"))
  (shadow 'land (find-package "XBOX")))
(unless (find-package "XBOX::LAND")
  (make-package "XBOX::LAND"))

(in-package "ROS")
;;//! \htmlinclude land.msg.html


(defclass xbox::land
  :super ros::object
  :slots (_fwd _rev _strl ))

(defmethod xbox::land
  (:init
   (&key
    ((:fwd __fwd) 0.0)
    ((:rev __rev) 0.0)
    ((:strl __strl) 0.0)
    )
   (send-super :init)
   (setq _fwd (float __fwd))
   (setq _rev (float __rev))
   (setq _strl (float __strl))
   self)
  (:fwd
   (&optional __fwd)
   (if __fwd (setq _fwd __fwd)) _fwd)
  (:rev
   (&optional __rev)
   (if __rev (setq _rev __rev)) _rev)
  (:strl
   (&optional __strl)
   (if __strl (setq _strl __strl)) _strl)
  (:serialization-length
   ()
   (+
    ;; float32 _fwd
    4
    ;; float32 _rev
    4
    ;; float32 _strl
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _fwd
       (sys::poke _fwd (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rev
       (sys::poke _rev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _strl
       (sys::poke _strl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _fwd
     (setq _fwd (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rev
     (setq _rev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _strl
     (setq _strl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get xbox::land :md5sum-) "edaf8b5219d5128d3faa0e4ddabb09c7")
(setf (get xbox::land :datatype-) "xbox/land")
(setf (get xbox::land :definition-)
      "# message file for land control commands

float32 fwd
float32 rev
float32 strl

")



(provide :xbox/land "edaf8b5219d5128d3faa0e4ddabb09c7")


