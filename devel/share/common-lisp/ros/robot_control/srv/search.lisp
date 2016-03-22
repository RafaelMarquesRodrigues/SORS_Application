; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude search-request.msg.html

(cl:defclass <search-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass search-request (<search-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <search-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'search-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<search-request> is deprecated: use robot_control-srv:search-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <search-request>) ostream)
  "Serializes a message object of type '<search-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <search-request>) istream)
  "Deserializes a message object of type '<search-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<search-request>)))
  "Returns string type for a service object of type '<search-request>"
  "robot_control/searchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'search-request)))
  "Returns string type for a service object of type 'search-request"
  "robot_control/searchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<search-request>)))
  "Returns md5sum for a message object of type '<search-request>"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'search-request)))
  "Returns md5sum for a message object of type 'search-request"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<search-request>)))
  "Returns full string definition for message of type '<search-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'search-request)))
  "Returns full string definition for message of type 'search-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <search-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <search-request>))
  "Converts a ROS message object to a list"
  (cl:list 'search-request
))
;//! \htmlinclude search-response.msg.html

(cl:defclass <search-response> (roslisp-msg-protocol:ros-message)
  ((knowledge
    :reader knowledge
    :initarg :knowledge
    :type cl:float
    :initform 0.0))
)

(cl:defclass search-response (<search-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <search-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'search-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<search-response> is deprecated: use robot_control-srv:search-response instead.")))

(cl:ensure-generic-function 'knowledge-val :lambda-list '(m))
(cl:defmethod knowledge-val ((m <search-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:knowledge-val is deprecated.  Use robot_control-srv:knowledge instead.")
  (knowledge m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <search-response>) ostream)
  "Serializes a message object of type '<search-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'knowledge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <search-response>) istream)
  "Deserializes a message object of type '<search-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'knowledge) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<search-response>)))
  "Returns string type for a service object of type '<search-response>"
  "robot_control/searchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'search-response)))
  "Returns string type for a service object of type 'search-response"
  "robot_control/searchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<search-response>)))
  "Returns md5sum for a message object of type '<search-response>"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'search-response)))
  "Returns md5sum for a message object of type 'search-response"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<search-response>)))
  "Returns full string definition for message of type '<search-response>"
  (cl:format cl:nil "float32 knowledge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'search-response)))
  "Returns full string definition for message of type 'search-response"
  (cl:format cl:nil "float32 knowledge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <search-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <search-response>))
  "Converts a ROS message object to a list"
  (cl:list 'search-response
    (cl:cons ':knowledge (knowledge msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'search)))
  'search-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'search)))
  'search-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'search)))
  "Returns string type for a service object of type '<search>"
  "robot_control/search")