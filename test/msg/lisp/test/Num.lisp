; Auto-generated. Do not edit!


(in-package test-msg)


;//! \htmlinclude Num.msg.html

(defclass <Num> (ros-message)
  ((num
    :reader num-val
    :initarg :num
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <Num>) ostream)
  "Serializes a message object of type '<Num>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'num)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'num)) ostream)
)
(defmethod deserialize ((msg <Num>) istream)
  "Deserializes a message object of type '<Num>"
  (setf (ldb (byte 8 0) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'num)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'num)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<Num>)))
  "Returns string type for a message object of type '<Num>"
  "test/Num")
(defmethod md5sum ((type (eql '<Num>)))
  "Returns md5sum for a message object of type '<Num>"
  "57d3c40ec3ac3754af76a83e6e73127a")
(defmethod message-definition ((type (eql '<Num>)))
  "Returns full string definition for message of type '<Num>"
  (format nil "int64 num~%~%~%"))
(defmethod serialization-length ((msg <Num>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <Num>))
  "Converts a ROS message object to a list"
  (list '<Num>
    (cons ':num (num-val msg))
))
