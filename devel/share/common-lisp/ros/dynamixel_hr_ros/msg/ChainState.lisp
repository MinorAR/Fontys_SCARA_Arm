; Auto-generated. Do not edit!


(cl:in-package dynamixel_hr_ros-msg)


;//! \htmlinclude ChainState.msg.html

(cl:defclass <ChainState> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (angle
    :reader angle
    :initarg :angle
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ChainState (<ChainState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChainState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChainState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dynamixel_hr_ros-msg:<ChainState> is deprecated: use dynamixel_hr_ros-msg:ChainState instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ChainState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hr_ros-msg:id-val is deprecated.  Use dynamixel_hr_ros-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <ChainState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dynamixel_hr_ros-msg:angle-val is deprecated.  Use dynamixel_hr_ros-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChainState>) ostream)
  "Serializes a message object of type '<ChainState>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'angle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChainState>) istream)
  "Deserializes a message object of type '<ChainState>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'id) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'id)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'angle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'angle)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChainState>)))
  "Returns string type for a message object of type '<ChainState>"
  "dynamixel_hr_ros/ChainState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChainState)))
  "Returns string type for a message object of type 'ChainState"
  "dynamixel_hr_ros/ChainState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChainState>)))
  "Returns md5sum for a message object of type '<ChainState>"
  "7771c1e7a4b4d36d3cc6bf12733a6fa0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChainState)))
  "Returns md5sum for a message object of type 'ChainState"
  "7771c1e7a4b4d36d3cc6bf12733a6fa0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChainState>)))
  "Returns full string definition for message of type '<ChainState>"
  (cl:format cl:nil "int8[] id~%float32[] angle~%#float32[] speed~%#bool[] moving~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChainState)))
  "Returns full string definition for message of type 'ChainState"
  (cl:format cl:nil "int8[] id~%float32[] angle~%#float32[] speed~%#bool[] moving~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChainState>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'id) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'angle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChainState>))
  "Converts a ROS message object to a list"
  (cl:list 'ChainState
    (cl:cons ':id (id msg))
    (cl:cons ':angle (angle msg))
))
