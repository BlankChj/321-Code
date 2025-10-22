
(cl:in-package :asdf)

(defsystem "kalman_filter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Vector3Stamped" :depends-on ("_package_Vector3Stamped"))
    (:file "_package_Vector3Stamped" :depends-on ("_package"))
  ))