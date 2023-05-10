
(cl:in-package :asdf)

(defsystem "navigation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Calibration" :depends-on ("_package_Calibration"))
    (:file "_package_Calibration" :depends-on ("_package"))
  ))