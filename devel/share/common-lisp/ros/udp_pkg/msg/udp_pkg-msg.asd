
(cl:in-package :asdf)

(defsystem "udp_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PositionVelocityAccel" :depends-on ("_package_PositionVelocityAccel"))
    (:file "_package_PositionVelocityAccel" :depends-on ("_package"))
  ))