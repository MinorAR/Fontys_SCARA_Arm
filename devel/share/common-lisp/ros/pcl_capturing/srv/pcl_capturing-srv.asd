
(cl:in-package :asdf)

(defsystem "pcl_capturing-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "position_request" :depends-on ("_package_position_request"))
    (:file "_package_position_request" :depends-on ("_package"))
  ))