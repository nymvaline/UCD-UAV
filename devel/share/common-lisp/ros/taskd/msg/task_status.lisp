; Auto-generated. Do not edit!


(cl:in-package taskd-msg)


;//! \htmlinclude task_status.msg.html

(cl:defclass <task_status> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (finished
    :reader finished
    :initarg :finished
    :type cl:fixnum
    :initform 0))
)

(cl:defclass task_status (<task_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <task_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'task_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name taskd-msg:<task_status> is deprecated: use taskd-msg:task_status instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <task_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader taskd-msg:header-val is deprecated.  Use taskd-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <task_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader taskd-msg:name-val is deprecated.  Use taskd-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <task_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader taskd-msg:finished-val is deprecated.  Use taskd-msg:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <task_status>) ostream)
  "Serializes a message object of type '<task_status>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finished)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <task_status>) istream)
  "Deserializes a message object of type '<task_status>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finished)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<task_status>)))
  "Returns string type for a message object of type '<task_status>"
  "taskd/task_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task_status)))
  "Returns string type for a message object of type 'task_status"
  "taskd/task_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task_status>)))
  "Returns md5sum for a message object of type '<task_status>"
  "4935533e3a4f5bb86056eb4d684e7c54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task_status)))
  "Returns md5sum for a message object of type 'task_status"
  "4935533e3a4f5bb86056eb4d684e7c54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task_status>)))
  "Returns full string definition for message of type '<task_status>"
  (cl:format cl:nil "Header header~%string name~%uint8 finished~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task_status)))
  "Returns full string definition for message of type 'task_status"
  (cl:format cl:nil "Header header~%string name~%uint8 finished~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <task_status>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <task_status>))
  "Converts a ROS message object to a list"
  (cl:list 'task_status
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':finished (finished msg))
))
