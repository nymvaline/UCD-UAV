;; Auto-generated. Do not edit!


(when (boundp 'taskd::status)
  (if (not (find-package "TASKD"))
    (make-package "TASKD"))
  (shadow 'status (find-package "TASKD")))
(unless (find-package "TASKD::STATUS")
  (make-package "TASKD::STATUS"))

(in-package "ROS")
;;//! \htmlinclude status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass taskd::status
  :super ros::object
  :slots (_header _name _finished ))

(defmethod taskd::status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:finished __finished) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _finished (round __finished))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:finished
   (&optional __finished)
   (if __finished (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint8 _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint8 _finished
       (write-byte _finished s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _finished
     (setq _finished (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get taskd::status :md5sum-) "4935533e3a4f5bb86056eb4d684e7c54")
(setf (get taskd::status :datatype-) "taskd/status")
(setf (get taskd::status :definition-)
      "Header header
string name
uint8 finished

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :taskd/status "4935533e3a4f5bb86056eb4d684e7c54")


