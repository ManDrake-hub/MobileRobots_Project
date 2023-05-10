; Auto-generated. Do not edit!


(cl:in-package navigation-srv)


;//! \htmlinclude Calibration-request.msg.html

(cl:defclass <Calibration-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Calibration-request (<Calibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Calibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Calibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<Calibration-request> is deprecated: use navigation-srv:Calibration-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Calibration-request>) ostream)
  "Serializes a message object of type '<Calibration-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Calibration-request>) istream)
  "Deserializes a message object of type '<Calibration-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Calibration-request>)))
  "Returns string type for a service object of type '<Calibration-request>"
  "navigation/CalibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibration-request)))
  "Returns string type for a service object of type 'Calibration-request"
  "navigation/CalibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Calibration-request>)))
  "Returns md5sum for a message object of type '<Calibration-request>"
  "a255970bffa18c4d56f14a3df609a6be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Calibration-request)))
  "Returns md5sum for a message object of type 'Calibration-request"
  "a255970bffa18c4d56f14a3df609a6be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Calibration-request>)))
  "Returns full string definition for message of type '<Calibration-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Calibration-request)))
  "Returns full string definition for message of type 'Calibration-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Calibration-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Calibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Calibration-request
))
;//! \htmlinclude Calibration-response.msg.html

(cl:defclass <Calibration-response> (roslisp-msg-protocol:ros-message)
  ((answer
    :reader answer
    :initarg :answer
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass Calibration-response (<Calibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Calibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Calibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-srv:<Calibration-response> is deprecated: use navigation-srv:Calibration-response instead.")))

(cl:ensure-generic-function 'answer-val :lambda-list '(m))
(cl:defmethod answer-val ((m <Calibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-srv:answer-val is deprecated.  Use navigation-srv:answer instead.")
  (answer m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Calibration-response>) ostream)
  "Serializes a message object of type '<Calibration-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'answer) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Calibration-response>) istream)
  "Deserializes a message object of type '<Calibration-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'answer) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Calibration-response>)))
  "Returns string type for a service object of type '<Calibration-response>"
  "navigation/CalibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibration-response)))
  "Returns string type for a service object of type 'Calibration-response"
  "navigation/CalibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Calibration-response>)))
  "Returns md5sum for a message object of type '<Calibration-response>"
  "a255970bffa18c4d56f14a3df609a6be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Calibration-response)))
  "Returns md5sum for a message object of type 'Calibration-response"
  "a255970bffa18c4d56f14a3df609a6be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Calibration-response>)))
  "Returns full string definition for message of type '<Calibration-response>"
  (cl:format cl:nil "std_msgs/String answer~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Calibration-response)))
  "Returns full string definition for message of type 'Calibration-response"
  (cl:format cl:nil "std_msgs/String answer~%~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Calibration-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'answer))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Calibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Calibration-response
    (cl:cons ':answer (answer msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Calibration)))
  'Calibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Calibration)))
  'Calibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibration)))
  "Returns string type for a service object of type '<Calibration>"
  "navigation/Calibration")