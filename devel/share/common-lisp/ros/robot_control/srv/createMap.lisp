; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude createMap-request.msg.html

(cl:defclass <createMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass createMap-request (<createMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <createMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'createMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<createMap-request> is deprecated: use robot_control-srv:createMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <createMap-request>) ostream)
  "Serializes a message object of type '<createMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <createMap-request>) istream)
  "Deserializes a message object of type '<createMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<createMap-request>)))
  "Returns string type for a service object of type '<createMap-request>"
  "robot_control/createMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createMap-request)))
  "Returns string type for a service object of type 'createMap-request"
  "robot_control/createMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<createMap-request>)))
  "Returns md5sum for a message object of type '<createMap-request>"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'createMap-request)))
  "Returns md5sum for a message object of type 'createMap-request"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<createMap-request>)))
  "Returns full string definition for message of type '<createMap-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'createMap-request)))
  "Returns full string definition for message of type 'createMap-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <createMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <createMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'createMap-request
))
;//! \htmlinclude createMap-response.msg.html

(cl:defclass <createMap-response> (roslisp-msg-protocol:ros-message)
  ((knowledge
    :reader knowledge
    :initarg :knowledge
    :type cl:float
    :initform 0.0))
)

(cl:defclass createMap-response (<createMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <createMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'createMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<createMap-response> is deprecated: use robot_control-srv:createMap-response instead.")))

(cl:ensure-generic-function 'knowledge-val :lambda-list '(m))
(cl:defmethod knowledge-val ((m <createMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:knowledge-val is deprecated.  Use robot_control-srv:knowledge instead.")
  (knowledge m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <createMap-response>) ostream)
  "Serializes a message object of type '<createMap-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'knowledge))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <createMap-response>) istream)
  "Deserializes a message object of type '<createMap-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'knowledge) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<createMap-response>)))
  "Returns string type for a service object of type '<createMap-response>"
  "robot_control/createMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createMap-response)))
  "Returns string type for a service object of type 'createMap-response"
  "robot_control/createMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<createMap-response>)))
  "Returns md5sum for a message object of type '<createMap-response>"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'createMap-response)))
  "Returns md5sum for a message object of type 'createMap-response"
  "8e5036d42e3183a686cafeab6c2d9883")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<createMap-response>)))
  "Returns full string definition for message of type '<createMap-response>"
  (cl:format cl:nil "float32 knowledge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'createMap-response)))
  "Returns full string definition for message of type 'createMap-response"
  (cl:format cl:nil "float32 knowledge~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <createMap-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <createMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'createMap-response
    (cl:cons ':knowledge (knowledge msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'createMap)))
  'createMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'createMap)))
  'createMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'createMap)))
  "Returns string type for a service object of type '<createMap>"
  "robot_control/createMap")