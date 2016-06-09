; Auto-generated. Do not edit!


(cl:in-package pcl_capturing-srv)


;//! \htmlinclude position_request-request.msg.html

(cl:defclass <position_request-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass position_request-request (<position_request-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <position_request-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'position_request-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_capturing-srv:<position_request-request> is deprecated: use pcl_capturing-srv:position_request-request instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <position_request-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_capturing-srv:pose-val is deprecated.  Use pcl_capturing-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <position_request-request>) ostream)
  "Serializes a message object of type '<position_request-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <position_request-request>) istream)
  "Deserializes a message object of type '<position_request-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<position_request-request>)))
  "Returns string type for a service object of type '<position_request-request>"
  "pcl_capturing/position_requestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request-request)))
  "Returns string type for a service object of type 'position_request-request"
  "pcl_capturing/position_requestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<position_request-request>)))
  "Returns md5sum for a message object of type '<position_request-request>"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'position_request-request)))
  "Returns md5sum for a message object of type 'position_request-request"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<position_request-request>)))
  "Returns full string definition for message of type '<position_request-request>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'position_request-request)))
  "Returns full string definition for message of type 'position_request-request"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <position_request-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <position_request-request>))
  "Converts a ROS message object to a list"
  (cl:list 'position_request-request
    (cl:cons ':pose (pose msg))
))
;//! \htmlinclude position_request-response.msg.html

(cl:defclass <position_request-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass position_request-response (<position_request-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <position_request-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'position_request-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pcl_capturing-srv:<position_request-response> is deprecated: use pcl_capturing-srv:position_request-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <position_request-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pcl_capturing-srv:success-val is deprecated.  Use pcl_capturing-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <position_request-response>) ostream)
  "Serializes a message object of type '<position_request-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <position_request-response>) istream)
  "Deserializes a message object of type '<position_request-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<position_request-response>)))
  "Returns string type for a service object of type '<position_request-response>"
  "pcl_capturing/position_requestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request-response)))
  "Returns string type for a service object of type 'position_request-response"
  "pcl_capturing/position_requestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<position_request-response>)))
  "Returns md5sum for a message object of type '<position_request-response>"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'position_request-response)))
  "Returns md5sum for a message object of type 'position_request-response"
  "b66f4c8c2fa81b98544cf0f3e10950ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<position_request-response>)))
  "Returns full string definition for message of type '<position_request-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'position_request-response)))
  "Returns full string definition for message of type 'position_request-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <position_request-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <position_request-response>))
  "Converts a ROS message object to a list"
  (cl:list 'position_request-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'position_request)))
  'position_request-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'position_request)))
  'position_request-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request)))
  "Returns string type for a service object of type '<position_request>"
  "pcl_capturing/position_request")