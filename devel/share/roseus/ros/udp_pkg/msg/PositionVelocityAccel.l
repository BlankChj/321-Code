;; Auto-generated. Do not edit!


(when (boundp 'udp_pkg::PositionVelocityAccel)
  (if (not (find-package "UDP_PKG"))
    (make-package "UDP_PKG"))
  (shadow 'PositionVelocityAccel (find-package "UDP_PKG")))
(unless (find-package "UDP_PKG::POSITIONVELOCITYACCEL")
  (make-package "UDP_PKG::POSITIONVELOCITYACCEL"))

(in-package "ROS")
;;//! \htmlinclude PositionVelocityAccel.msg.html


(defclass udp_pkg::PositionVelocityAccel
  :super ros::object
  :slots (_frame_id _stamp _x_pos _y_pos _z_pos _x_ori _y_ori _z_ori _w_ori _x_vel _y_vel _z_vel _x_acc _y_acc _z_acc ))

(defmethod udp_pkg::PositionVelocityAccel
  (:init
   (&key
    ((:frame_id __frame_id) "")
    ((:stamp __stamp) 0.0)
    ((:x_pos __x_pos) 0.0)
    ((:y_pos __y_pos) 0.0)
    ((:z_pos __z_pos) 0.0)
    ((:x_ori __x_ori) 0.0)
    ((:y_ori __y_ori) 0.0)
    ((:z_ori __z_ori) 0.0)
    ((:w_ori __w_ori) 0.0)
    ((:x_vel __x_vel) 0.0)
    ((:y_vel __y_vel) 0.0)
    ((:z_vel __z_vel) 0.0)
    ((:x_acc __x_acc) 0.0)
    ((:y_acc __y_acc) 0.0)
    ((:z_acc __z_acc) 0.0)
    )
   (send-super :init)
   (setq _frame_id (string __frame_id))
   (setq _stamp (float __stamp))
   (setq _x_pos (float __x_pos))
   (setq _y_pos (float __y_pos))
   (setq _z_pos (float __z_pos))
   (setq _x_ori (float __x_ori))
   (setq _y_ori (float __y_ori))
   (setq _z_ori (float __z_ori))
   (setq _w_ori (float __w_ori))
   (setq _x_vel (float __x_vel))
   (setq _y_vel (float __y_vel))
   (setq _z_vel (float __z_vel))
   (setq _x_acc (float __x_acc))
   (setq _y_acc (float __y_acc))
   (setq _z_acc (float __z_acc))
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:x_pos
   (&optional __x_pos)
   (if __x_pos (setq _x_pos __x_pos)) _x_pos)
  (:y_pos
   (&optional __y_pos)
   (if __y_pos (setq _y_pos __y_pos)) _y_pos)
  (:z_pos
   (&optional __z_pos)
   (if __z_pos (setq _z_pos __z_pos)) _z_pos)
  (:x_ori
   (&optional __x_ori)
   (if __x_ori (setq _x_ori __x_ori)) _x_ori)
  (:y_ori
   (&optional __y_ori)
   (if __y_ori (setq _y_ori __y_ori)) _y_ori)
  (:z_ori
   (&optional __z_ori)
   (if __z_ori (setq _z_ori __z_ori)) _z_ori)
  (:w_ori
   (&optional __w_ori)
   (if __w_ori (setq _w_ori __w_ori)) _w_ori)
  (:x_vel
   (&optional __x_vel)
   (if __x_vel (setq _x_vel __x_vel)) _x_vel)
  (:y_vel
   (&optional __y_vel)
   (if __y_vel (setq _y_vel __y_vel)) _y_vel)
  (:z_vel
   (&optional __z_vel)
   (if __z_vel (setq _z_vel __z_vel)) _z_vel)
  (:x_acc
   (&optional __x_acc)
   (if __x_acc (setq _x_acc __x_acc)) _x_acc)
  (:y_acc
   (&optional __y_acc)
   (if __y_acc (setq _y_acc __y_acc)) _y_acc)
  (:z_acc
   (&optional __z_acc)
   (if __z_acc (setq _z_acc __z_acc)) _z_acc)
  (:serialization-length
   ()
   (+
    ;; string _frame_id
    4 (length _frame_id)
    ;; float64 _stamp
    8
    ;; float64 _x_pos
    8
    ;; float64 _y_pos
    8
    ;; float64 _z_pos
    8
    ;; float64 _x_ori
    8
    ;; float64 _y_ori
    8
    ;; float64 _z_ori
    8
    ;; float64 _w_ori
    8
    ;; float64 _x_vel
    8
    ;; float64 _y_vel
    8
    ;; float64 _z_vel
    8
    ;; float64 _x_acc
    8
    ;; float64 _y_acc
    8
    ;; float64 _z_acc
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _frame_id
       (write-long (length _frame_id) s) (princ _frame_id s)
     ;; float64 _stamp
       (sys::poke _stamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_pos
       (sys::poke _x_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_pos
       (sys::poke _y_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_pos
       (sys::poke _z_pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_ori
       (sys::poke _x_ori (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_ori
       (sys::poke _y_ori (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_ori
       (sys::poke _z_ori (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _w_ori
       (sys::poke _w_ori (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_vel
       (sys::poke _x_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_vel
       (sys::poke _y_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_vel
       (sys::poke _z_vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_acc
       (sys::poke _x_acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_acc
       (sys::poke _y_acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_acc
       (sys::poke _z_acc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _stamp
     (setq _stamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_pos
     (setq _x_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_pos
     (setq _y_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_pos
     (setq _z_pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_ori
     (setq _x_ori (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_ori
     (setq _y_ori (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_ori
     (setq _z_ori (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _w_ori
     (setq _w_ori (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_vel
     (setq _x_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_vel
     (setq _y_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_vel
     (setq _z_vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_acc
     (setq _x_acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_acc
     (setq _y_acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_acc
     (setq _z_acc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get udp_pkg::PositionVelocityAccel :md5sum-) "b1ec75ae739361310f2b2bc1600571ae")
(setf (get udp_pkg::PositionVelocityAccel :datatype-) "udp_pkg/PositionVelocityAccel")
(setf (get udp_pkg::PositionVelocityAccel :definition-)
      "string frame_id
float64 stamp
float64 x_pos
float64 y_pos
float64 z_pos
float64 x_ori
float64 y_ori
float64 z_ori
float64 w_ori
float64 x_vel
float64 y_vel
float64 z_vel
float64 x_acc
float64 y_acc
float64 z_acc

")



(provide :udp_pkg/PositionVelocityAccel "b1ec75ae739361310f2b2bc1600571ae")


