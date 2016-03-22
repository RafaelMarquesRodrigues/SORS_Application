; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude addToMap-request.msg.html

(cl:defclass <addToMap-request> (roslisp-msg-protocol:ros-message)
  ((start_x
    :reader start_x
    :initarg :start_x
    :type cl:float
    :initform 0.0)
   (start_y
    :reader start_y
    :initarg :start_y
    :type cl:float
    :initform 0.0)
   (wall_x
    :reader wall_x
    :initarg :wall_x
    :type cl:float
    :initform 0.0)
   (wall_y
    :reader wall_y
    :initarg :wall_y
    :type cl:float
    :initform 0.0)
   (inc_x
    :reader inc_x
    :initarg :inc_x
    :type cl:float
    :initform 0.0)
   (inc_y
    :reader inc_y
    :initarg :inc_y
    :type cl:float
    :initform 0.0)
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0))
)

(cl:defclass addToMap-request (<addToMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <addToMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'addToMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<addToMap-request> is deprecated: use robot_control-srv:addToMap-request instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:start_x-val is deprecated.  Use robot_control-srv:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:start_y-val is deprecated.  Use robot_control-srv:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'wall_x-val :lambda-list '(m))
(cl:defmethod wall_x-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:wall_x-val is deprecated.  Use robot_control-srv:wall_x instead.")
  (wall_x m))

(cl:ensure-generic-function 'wall_y-val :lambda-list '(m))
(cl:defmethod wall_y-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:wall_y-val is deprecated.  Use robot_control-srv:wall_y instead.")
  (wall_y m))

(cl:ensure-generic-function 'inc_x-val :lambda-list '(m))
(cl:defmethod inc_x-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:inc_x-val is deprecated.  Use robot_control-srv:inc_x instead.")
  (inc_x m))

(cl:ensure-generic-function 'inc_y-val :lambda-list '(m))
(cl:defmethod inc_y-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:inc_y-val is deprecated.  Use robot_control-srv:inc_y instead.")
  (inc_y m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <addToMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:range-val is deprecated.  Use robot_control-srv:range instead.")
  (range m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <addToMap-request>) ostream)
  "Serializes a message object of type '<addToMap-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'start_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wall_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wall_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'inc_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'inc_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <addToMap-request>) istream)
  "Deserializes a message object of type '<addToMap-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'start_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wall_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wall_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inc_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inc_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<addToMap-request>)))
  "Returns string type for a service object of type '<addToMap-request>"
  "robot_control/addToMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addToMap-request)))
  "Returns string type for a service object of type 'addToMap-request"
  "robot_control/addToMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<addToMap-request>)))
  "Returns md5sum for a message object of type '<addToMap-request>"
  "b0f29972a30c9830b340e4bab99692b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'addToMap-request)))
  "Returns md5sum for a message object of type 'addToMap-request"
  "b0f29972a30c9830b340e4bab99692b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<addToMap-request>)))
  "Returns full string definition for message of type '<addToMap-request>"
  (cl:format cl:nil "float64 start_x~%float64 start_y~%float64 wall_x~%float64 wall_y~%float64 inc_x~%float64 inc_y~%float64 range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'addToMap-request)))
  "Returns full string definition for message of type 'addToMap-request"
  (cl:format cl:nil "float64 start_x~%float64 start_y~%float64 wall_x~%float64 wall_y~%float64 inc_x~%float64 inc_y~%float64 range~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <addToMap-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <addToMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'addToMap-request
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':wall_x (wall_x msg))
    (cl:cons ':wall_y (wall_y msg))
    (cl:cons ':inc_x (inc_x msg))
    (cl:cons ':inc_y (inc_y msg))
    (cl:cons ':range (range msg))
))
;//! \htmlinclude addToMap-response.msg.html

(cl:defclass <addToMap-response> (roslisp-msg-protocol:ros-message)
  ((added
    :reader added
    :initarg :added
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass addToMap-response (<addToMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <addToMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'addToMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<addToMap-response> is deprecated: use robot_control-srv:addToMap-response instead.")))

(cl:ensure-generic-function 'added-val :lambda-list '(m))
(cl:defmethod added-val ((m <addToMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:added-val is deprecated.  Use robot_control-srv:added instead.")
  (added m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <addToMap-response>) ostream)
  "Serializes a message object of type '<addToMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'added) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <addToMap-response>) istream)
  "Deserializes a message object of type '<addToMap-response>"
    (cl:setf (cl:slot-value msg 'added) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<addToMap-response>)))
  "Returns string type for a service object of type '<addToMap-response>"
  "robot_control/addToMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addToMap-response)))
  "Returns string type for a service object of type 'addToMap-response"
  "robot_control/addToMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<addToMap-response>)))
  "Returns md5sum for a message object of type '<addToMap-response>"
  "b0f29972a30c9830b340e4bab99692b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'addToMap-response)))
  "Returns md5sum for a message object of type 'addToMap-response"
  "b0f29972a30c9830b340e4bab99692b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<addToMap-response>)))
  "Returns full string definition for message of type '<addToMap-response>"
  (cl:format cl:nil "bool added~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'addToMap-response)))
  "Returns full string definition for message of type 'addToMap-response"
  (cl:format cl:nil "bool added~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <addToMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <addToMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'addToMap-response
    (cl:cons ':added (added msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'addToMap)))
  'addToMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'addToMap)))
  'addToMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'addToMap)))
  "Returns string type for a service object of type '<addToMap>"
  "robot_control/addToMap")