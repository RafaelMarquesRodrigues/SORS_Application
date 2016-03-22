; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude getPositions-request.msg.html

(cl:defclass <getPositions-request> (roslisp-msg-protocol:ros-message)
  ((my_x
    :reader my_x
    :initarg :my_x
    :type cl:float
    :initform 0.0)
   (my_y
    :reader my_y
    :initarg :my_y
    :type cl:float
    :initform 0.0)
   (my_id
    :reader my_id
    :initarg :my_id
    :type cl:integer
    :initform 0)
   (has_id
    :reader has_id
    :initarg :has_id
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass getPositions-request (<getPositions-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getPositions-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getPositions-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<getPositions-request> is deprecated: use robot_control-srv:getPositions-request instead.")))

(cl:ensure-generic-function 'my_x-val :lambda-list '(m))
(cl:defmethod my_x-val ((m <getPositions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:my_x-val is deprecated.  Use robot_control-srv:my_x instead.")
  (my_x m))

(cl:ensure-generic-function 'my_y-val :lambda-list '(m))
(cl:defmethod my_y-val ((m <getPositions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:my_y-val is deprecated.  Use robot_control-srv:my_y instead.")
  (my_y m))

(cl:ensure-generic-function 'my_id-val :lambda-list '(m))
(cl:defmethod my_id-val ((m <getPositions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:my_id-val is deprecated.  Use robot_control-srv:my_id instead.")
  (my_id m))

(cl:ensure-generic-function 'has_id-val :lambda-list '(m))
(cl:defmethod has_id-val ((m <getPositions-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:has_id-val is deprecated.  Use robot_control-srv:has_id instead.")
  (has_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getPositions-request>) ostream)
  "Serializes a message object of type '<getPositions-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'my_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'my_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'my_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_id) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getPositions-request>) istream)
  "Deserializes a message object of type '<getPositions-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'my_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'my_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'my_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'has_id) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getPositions-request>)))
  "Returns string type for a service object of type '<getPositions-request>"
  "robot_control/getPositionsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getPositions-request)))
  "Returns string type for a service object of type 'getPositions-request"
  "robot_control/getPositionsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getPositions-request>)))
  "Returns md5sum for a message object of type '<getPositions-request>"
  "70953a0a6fffaa23e9167aef60c438f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getPositions-request)))
  "Returns md5sum for a message object of type 'getPositions-request"
  "70953a0a6fffaa23e9167aef60c438f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getPositions-request>)))
  "Returns full string definition for message of type '<getPositions-request>"
  (cl:format cl:nil "float64 my_x~%float64 my_y~%int32 my_id~%bool has_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getPositions-request)))
  "Returns full string definition for message of type 'getPositions-request"
  (cl:format cl:nil "float64 my_x~%float64 my_y~%int32 my_id~%bool has_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getPositions-request>))
  (cl:+ 0
     8
     8
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getPositions-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getPositions-request
    (cl:cons ':my_x (my_x msg))
    (cl:cons ':my_y (my_y msg))
    (cl:cons ':my_id (my_id msg))
    (cl:cons ':has_id (has_id msg))
))
;//! \htmlinclude getPositions-response.msg.html

(cl:defclass <getPositions-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (y
    :reader y
    :initarg :y
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass getPositions-response (<getPositions-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getPositions-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getPositions-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<getPositions-response> is deprecated: use robot_control-srv:getPositions-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <getPositions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:x-val is deprecated.  Use robot_control-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <getPositions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:y-val is deprecated.  Use robot_control-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <getPositions-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:id-val is deprecated.  Use robot_control-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getPositions-response>) ostream)
  "Serializes a message object of type '<getPositions-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'y))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getPositions-response>) istream)
  "Deserializes a message object of type '<getPositions-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getPositions-response>)))
  "Returns string type for a service object of type '<getPositions-response>"
  "robot_control/getPositionsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getPositions-response)))
  "Returns string type for a service object of type 'getPositions-response"
  "robot_control/getPositionsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getPositions-response>)))
  "Returns md5sum for a message object of type '<getPositions-response>"
  "70953a0a6fffaa23e9167aef60c438f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getPositions-response)))
  "Returns md5sum for a message object of type 'getPositions-response"
  "70953a0a6fffaa23e9167aef60c438f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getPositions-response>)))
  "Returns full string definition for message of type '<getPositions-response>"
  (cl:format cl:nil "float64[] x~%float64[] y~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getPositions-response)))
  "Returns full string definition for message of type 'getPositions-response"
  (cl:format cl:nil "float64[] x~%float64[] y~%int32 id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getPositions-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getPositions-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getPositions-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':id (id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getPositions)))
  'getPositions-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getPositions)))
  'getPositions-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getPositions)))
  "Returns string type for a service object of type '<getPositions>"
  "robot_control/getPositions")