; Auto-generated. Do not edit!


(cl:in-package rm_msgs-msg)


;//! \htmlinclude Ort_Teach.msg.html

(cl:defclass <Ort_Teach> (roslisp-msg-protocol:ros-message)
  ((teach_type
    :reader teach_type
    :initarg :teach_type
    :type cl:string
    :initform "")
   (direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform "")
   (v
    :reader v
    :initarg :v
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Ort_Teach (<Ort_Teach>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ort_Teach>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ort_Teach)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rm_msgs-msg:<Ort_Teach> is deprecated: use rm_msgs-msg:Ort_Teach instead.")))

(cl:ensure-generic-function 'teach_type-val :lambda-list '(m))
(cl:defmethod teach_type-val ((m <Ort_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:teach_type-val is deprecated.  Use rm_msgs-msg:teach_type instead.")
  (teach_type m))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <Ort_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:direction-val is deprecated.  Use rm_msgs-msg:direction instead.")
  (direction m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <Ort_Teach>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rm_msgs-msg:v-val is deprecated.  Use rm_msgs-msg:v instead.")
  (v m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ort_Teach>) ostream)
  "Serializes a message object of type '<Ort_Teach>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'teach_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'teach_type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
  (cl:let* ((signed (cl:slot-value msg 'v)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ort_Teach>) istream)
  "Deserializes a message object of type '<Ort_Teach>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'teach_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'teach_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'v) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ort_Teach>)))
  "Returns string type for a message object of type '<Ort_Teach>"
  "rm_msgs/Ort_Teach")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ort_Teach)))
  "Returns string type for a message object of type 'Ort_Teach"
  "rm_msgs/Ort_Teach")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ort_Teach>)))
  "Returns md5sum for a message object of type '<Ort_Teach>"
  "34da3e35edafae2adfbdcd46acdb6bd9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ort_Teach)))
  "Returns md5sum for a message object of type 'Ort_Teach"
  "34da3e35edafae2adfbdcd46acdb6bd9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ort_Teach>)))
  "Returns full string definition for message of type '<Ort_Teach>"
  (cl:format cl:nil "string teach_type~%string direction~%int16 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ort_Teach)))
  "Returns full string definition for message of type 'Ort_Teach"
  (cl:format cl:nil "string teach_type~%string direction~%int16 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ort_Teach>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'teach_type))
     4 (cl:length (cl:slot-value msg 'direction))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ort_Teach>))
  "Converts a ROS message object to a list"
  (cl:list 'Ort_Teach
    (cl:cons ':teach_type (teach_type msg))
    (cl:cons ':direction (direction msg))
    (cl:cons ':v (v msg))
))
