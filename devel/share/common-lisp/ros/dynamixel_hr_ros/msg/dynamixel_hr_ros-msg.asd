
(cl:in-package :asdf)

(defsystem "dynamixel_hr_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChainState" :depends-on ("_package_ChainState"))
    (:file "_package_ChainState" :depends-on ("_package"))
    (:file "CommandPosition" :depends-on ("_package_CommandPosition"))
    (:file "_package_CommandPosition" :depends-on ("_package"))
  ))