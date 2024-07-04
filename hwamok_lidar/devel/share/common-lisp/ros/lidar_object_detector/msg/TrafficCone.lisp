; Auto-generated. Do not edit!


(cl:in-package lidar_object_detector-msg)


;//! \htmlinclude TrafficCone.msg.html

(cl:defclass <TrafficCone> (roslisp-msg-protocol:ros-message)
  ((objectCounts
    :reader objectCounts
    :initarg :objectCounts
    :type cl:integer
    :initform 0)
   (centerX
    :reader centerX
    :initarg :centerX
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0))
   (centerY
    :reader centerY
    :initarg :centerY
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0))
   (centerZ
    :reader centerZ
    :initarg :centerZ
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0))
   (lengthX
    :reader lengthX
    :initarg :lengthX
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0))
   (lengthY
    :reader lengthY
    :initarg :lengthY
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0))
   (lengthZ
    :reader lengthZ
    :initarg :lengthZ
    :type (cl:vector cl:float)
   :initform (cl:make-array 100 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass TrafficCone (<TrafficCone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficCone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficCone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_object_detector-msg:<TrafficCone> is deprecated: use lidar_object_detector-msg:TrafficCone instead.")))

(cl:ensure-generic-function 'objectCounts-val :lambda-list '(m))
(cl:defmethod objectCounts-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:objectCounts-val is deprecated.  Use lidar_object_detector-msg:objectCounts instead.")
  (objectCounts m))

(cl:ensure-generic-function 'centerX-val :lambda-list '(m))
(cl:defmethod centerX-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:centerX-val is deprecated.  Use lidar_object_detector-msg:centerX instead.")
  (centerX m))

(cl:ensure-generic-function 'centerY-val :lambda-list '(m))
(cl:defmethod centerY-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:centerY-val is deprecated.  Use lidar_object_detector-msg:centerY instead.")
  (centerY m))

(cl:ensure-generic-function 'centerZ-val :lambda-list '(m))
(cl:defmethod centerZ-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:centerZ-val is deprecated.  Use lidar_object_detector-msg:centerZ instead.")
  (centerZ m))

(cl:ensure-generic-function 'lengthX-val :lambda-list '(m))
(cl:defmethod lengthX-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:lengthX-val is deprecated.  Use lidar_object_detector-msg:lengthX instead.")
  (lengthX m))

(cl:ensure-generic-function 'lengthY-val :lambda-list '(m))
(cl:defmethod lengthY-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:lengthY-val is deprecated.  Use lidar_object_detector-msg:lengthY instead.")
  (lengthY m))

(cl:ensure-generic-function 'lengthZ-val :lambda-list '(m))
(cl:defmethod lengthZ-val ((m <TrafficCone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_object_detector-msg:lengthZ-val is deprecated.  Use lidar_object_detector-msg:lengthZ instead.")
  (lengthZ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficCone>) ostream)
  "Serializes a message object of type '<TrafficCone>"
  (cl:let* ((signed (cl:slot-value msg 'objectCounts)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'centerX))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'centerY))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'centerZ))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lengthX))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lengthY))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'lengthZ))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficCone>) istream)
  "Deserializes a message object of type '<TrafficCone>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'objectCounts) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'centerX) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'centerX)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'centerY) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'centerY)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'centerZ) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'centerZ)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lengthX) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'lengthX)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lengthY) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'lengthY)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'lengthZ) (cl:make-array 100))
  (cl:let ((vals (cl:slot-value msg 'lengthZ)))
    (cl:dotimes (i 100)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficCone>)))
  "Returns string type for a message object of type '<TrafficCone>"
  "lidar_object_detector/TrafficCone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficCone)))
  "Returns string type for a message object of type 'TrafficCone"
  "lidar_object_detector/TrafficCone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficCone>)))
  "Returns md5sum for a message object of type '<TrafficCone>"
  "fcb703df87d24291d755127aee75bb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficCone)))
  "Returns md5sum for a message object of type 'TrafficCone"
  "fcb703df87d24291d755127aee75bb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficCone>)))
  "Returns full string definition for message of type '<TrafficCone>"
  (cl:format cl:nil "int32 objectCounts~%float64[100] centerX~%float64[100] centerY~%float64[100] centerZ~%float64[100] lengthX~%float64[100] lengthY~%float64[100] lengthZ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficCone)))
  "Returns full string definition for message of type 'TrafficCone"
  (cl:format cl:nil "int32 objectCounts~%float64[100] centerX~%float64[100] centerY~%float64[100] centerZ~%float64[100] lengthX~%float64[100] lengthY~%float64[100] lengthZ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficCone>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'centerX) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'centerY) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'centerZ) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lengthX) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lengthY) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'lengthZ) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficCone>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficCone
    (cl:cons ':objectCounts (objectCounts msg))
    (cl:cons ':centerX (centerX msg))
    (cl:cons ':centerY (centerY msg))
    (cl:cons ':centerZ (centerZ msg))
    (cl:cons ':lengthX (lengthX msg))
    (cl:cons ':lengthY (lengthY msg))
    (cl:cons ':lengthZ (lengthZ msg))
))
