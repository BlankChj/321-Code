;; Auto-generated. Do not edit!


(when (boundp 'kalman_filter::Vector3Stamped)
  (if (not (find-package "KALMAN_FILTER"))
    (make-package "KALMAN_FILTER"))
  (shadow 'Vector3Stamped (find-package "KALMAN_FILTER")))
(unless (find-package "KALMAN_FILTER::VECTOR3STAMPED")
  (make-package "KALMAN_FILTER::VECTOR3STAMPED"))

(in-package "ROS")
;;//! \htmlinclude Vector3Stamped.msg.html


(defclass kalman_filter::Vector3Stamped
  :super ros::object
  :slots (_time _x _y _z ))

(defmethod kalman_filter::Vector3Stamped
  (:init
   (&key
    ((:time __time) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _time (float __time))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float64 _time
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get kalman_filter::Vector3Stamped :md5sum-) "8ed4dcb6382e6a419737c7a7d2a6e517")
(setf (get kalman_filter::Vector3Stamped :datatype-) "kalman_filter/Vector3Stamped")
(setf (get kalman_filter::Vector3Stamped :definition-)
      "float64 time
float64 x
float64 y
float64 z
")



(provide :kalman_filter/Vector3Stamped "8ed4dcb6382e6a419737c7a7d2a6e517")


