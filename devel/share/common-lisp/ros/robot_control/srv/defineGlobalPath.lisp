; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude defineGlobalPath-request.msg.html

(cl:defclass <defineGlobalPath-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (destiny_x
    :reader destiny_x
    :initarg :destiny_x
    :type cl:float
    :initform 0.0)
   (destiny_y
    :reader destiny_y
    :initarg :destiny_y
    :type cl:float
    :initform 0.0)
   (cell_size
    :reader cell_size
    :initarg :cell_size
    :type cl:float
    :initform 0.0)
   (map
    :reader map
    :initarg :map
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass defineGlobalPath-request (<defineGlobalPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <defineGlobalPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'defineGlobalPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<defineGlobalPath-request> is deprecated: use robot_control-srv:defineGlobalPath-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:x-val is deprecated.  Use robot_control-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:y-val is deprecated.  Use robot_control-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'destiny_x-val :lambda-list '(m))
(cl:defmethod destiny_x-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:destiny_x-val is deprecated.  Use robot_control-srv:destiny_x instead.")
  (destiny_x m))

(cl:ensure-generic-function 'destiny_y-val :lambda-list '(m))
(cl:defmethod destiny_y-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:destiny_y-val is deprecated.  Use robot_control-srv:destiny_y instead.")
  (destiny_y m))

(cl:ensure-generic-function 'cell_size-val :lambda-list '(m))
(cl:defmethod cell_size-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:cell_size-val is deprecated.  Use robot_control-srv:cell_size instead.")
  (cell_size m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <defineGlobalPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:map-val is deprecated.  Use robot_control-srv:map instead.")
  (map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <defineGlobalPath-request>) ostream)
  "Serializes a message object of type '<defineGlobalPath-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'destiny_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'destiny_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cell_size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'map))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <defineGlobalPath-request>) istream)
  "Deserializes a message object of type '<defineGlobalPath-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'destiny_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'destiny_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cell_size) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<defineGlobalPath-request>)))
  "Returns string type for a service object of type '<defineGlobalPath-request>"
  "robot_control/defineGlobalPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'defineGlobalPath-request)))
  "Returns string type for a service object of type 'defineGlobalPath-request"
  "robot_control/defineGlobalPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<defineGlobalPath-request>)))
  "Returns md5sum for a message object of type '<defineGlobalPath-request>"
  "4fe135cc34bd036b473beef806eb4cd1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'defineGlobalPath-request)))
  "Returns md5sum for a message object of type 'defineGlobalPath-request"
  "4fe135cc34bd036b473beef806eb4cd1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<defineGlobalPath-request>)))
  "Returns full string definition for message of type '<defineGlobalPath-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 destiny_x~%float64 destiny_y~%float32 cell_size~%char[] map~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'defineGlobalPath-request)))
  "Returns full string definition for message of type 'defineGlobalPath-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 destiny_x~%float64 destiny_y~%float32 cell_size~%char[] map~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <defineGlobalPath-request>))
  (cl:+ 0
     8
     8
     8
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <defineGlobalPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'defineGlobalPath-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':destiny_x (destiny_x msg))
    (cl:cons ':destiny_y (destiny_y msg))
    (cl:cons ':cell_size (cell_size msg))
    (cl:cons ':map (map msg))
))
;//! \htmlinclude defineGlobalPath-response.msg.html

(cl:defclass <defineGlobalPath-response> (roslisp-msg-protocol:ros-message)
  ((x_path
    :reader x_path
    :initarg :x_path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (y_path
    :reader y_path
    :initarg :y_path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass defineGlobalPath-response (<defineGlobalPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <defineGlobalPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'defineGlobalPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<defineGlobalPath-response> is deprecated: use robot_control-srv:defineGlobalPath-response instead.")))

(cl:ensure-generic-function 'x_path-val :lambda-list '(m))
(cl:defmethod x_path-val ((m <defineGlobalPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:x_path-val is deprecated.  Use robot_control-srv:x_path instead.")
  (x_path m))

(cl:ensure-generic-function 'y_path-val :lambda-list '(m))
(cl:defmethod y_path-val ((m <defineGlobalPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:y_path-val is deprecated.  Use robot_control-srv:y_path instead.")
  (y_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <defineGlobalPath-response>) ostream)
  "Serializes a message object of type '<defineGlobalPath-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_path))))
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
   (cl:slot-value msg 'x_path))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y_path))))
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
   (cl:slot-value msg 'y_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <defineGlobalPath-response>) istream)
  "Deserializes a message object of type '<defineGlobalPath-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_path)))
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
  (cl:setf (cl:slot-value msg 'y_path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y_path)))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<defineGlobalPath-response>)))
  "Returns string type for a service object of type '<defineGlobalPath-response>"
  "robot_control/defineGlobalPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'defineGlobalPath-response)))
  "Returns string type for a service object of type 'defineGlobalPath-response"
  "robot_control/defineGlobalPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<defineGlobalPath-response>)))
  "Returns md5sum for a message object of type '<defineGlobalPath-response>"
  "4fe135cc34bd036b473beef806eb4cd1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'defineGlobalPath-response)))
  "Returns md5sum for a message object of type 'defineGlobalPath-response"
  "4fe135cc34bd036b473beef806eb4cd1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<defineGlobalPath-response>)))
  "Returns full string definition for message of type '<defineGlobalPath-response>"
  (cl:format cl:nil "float64[] x_path~%float64[] y_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'defineGlobalPath-response)))
  "Returns full string definition for message of type 'defineGlobalPath-response"
  (cl:format cl:nil "float64[] x_path~%float64[] y_path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <defineGlobalPath-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y_path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <defineGlobalPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'defineGlobalPath-response
    (cl:cons ':x_path (x_path msg))
    (cl:cons ':y_path (y_path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'defineGlobalPath)))
  'defineGlobalPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'defineGlobalPath)))
  'defineGlobalPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'defineGlobalPath)))
  "Returns string type for a service object of type '<defineGlobalPath>"
  "robot_control/defineGlobalPath")