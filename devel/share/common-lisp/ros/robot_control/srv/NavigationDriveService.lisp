; Auto-generated. Do not edit!


(cl:in-package robot_control-srv)


;//! \htmlinclude NavigationDriveService-request.msg.html

(cl:defclass <NavigationDriveService-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass NavigationDriveService-request (<NavigationDriveService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigationDriveService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigationDriveService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<NavigationDriveService-request> is deprecated: use robot_control-srv:NavigationDriveService-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigationDriveService-request>) ostream)
  "Serializes a message object of type '<NavigationDriveService-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigationDriveService-request>) istream)
  "Deserializes a message object of type '<NavigationDriveService-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigationDriveService-request>)))
  "Returns string type for a service object of type '<NavigationDriveService-request>"
  "robot_control/NavigationDriveServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationDriveService-request)))
  "Returns string type for a service object of type 'NavigationDriveService-request"
  "robot_control/NavigationDriveServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigationDriveService-request>)))
  "Returns md5sum for a message object of type '<NavigationDriveService-request>"
  "f9ac4e286e7f89fc602116455cd26e68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigationDriveService-request)))
  "Returns md5sum for a message object of type 'NavigationDriveService-request"
  "f9ac4e286e7f89fc602116455cd26e68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigationDriveService-request>)))
  "Returns full string definition for message of type '<NavigationDriveService-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigationDriveService-request)))
  "Returns full string definition for message of type 'NavigationDriveService-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigationDriveService-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigationDriveService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigationDriveService-request
))
;//! \htmlinclude NavigationDriveService-response.msg.html

(cl:defclass <NavigationDriveService-response> (roslisp-msg-protocol:ros-message)
  ((end
    :reader end
    :initarg :end
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass NavigationDriveService-response (<NavigationDriveService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigationDriveService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigationDriveService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_control-srv:<NavigationDriveService-response> is deprecated: use robot_control-srv:NavigationDriveService-response instead.")))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <NavigationDriveService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_control-srv:end-val is deprecated.  Use robot_control-srv:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigationDriveService-response>) ostream)
  "Serializes a message object of type '<NavigationDriveService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'end) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigationDriveService-response>) istream)
  "Deserializes a message object of type '<NavigationDriveService-response>"
    (cl:setf (cl:slot-value msg 'end) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigationDriveService-response>)))
  "Returns string type for a service object of type '<NavigationDriveService-response>"
  "robot_control/NavigationDriveServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationDriveService-response)))
  "Returns string type for a service object of type 'NavigationDriveService-response"
  "robot_control/NavigationDriveServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigationDriveService-response>)))
  "Returns md5sum for a message object of type '<NavigationDriveService-response>"
  "f9ac4e286e7f89fc602116455cd26e68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigationDriveService-response)))
  "Returns md5sum for a message object of type 'NavigationDriveService-response"
  "f9ac4e286e7f89fc602116455cd26e68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigationDriveService-response>)))
  "Returns full string definition for message of type '<NavigationDriveService-response>"
  (cl:format cl:nil "bool end~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigationDriveService-response)))
  "Returns full string definition for message of type 'NavigationDriveService-response"
  (cl:format cl:nil "bool end~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigationDriveService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigationDriveService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigationDriveService-response
    (cl:cons ':end (end msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'NavigationDriveService)))
  'NavigationDriveService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'NavigationDriveService)))
  'NavigationDriveService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationDriveService)))
  "Returns string type for a service object of type '<NavigationDriveService>"
  "robot_control/NavigationDriveService")