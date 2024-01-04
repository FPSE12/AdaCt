; Auto-generated. Do not edit!


(cl:in-package AdaCt-msg)


;//! \htmlinclude cloud_info.msg.html

(cl:defclass <cloud_info> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (timeStart
    :reader timeStart
    :initarg :timeStart
    :type cl:float
    :initform 0.0)
   (timeEnd
    :reader timeEnd
    :initarg :timeEnd
    :type cl:float
    :initform 0.0)
   (startRingIndex
    :reader startRingIndex
    :initarg :startRingIndex
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (endRingIndex
    :reader endRingIndex
    :initarg :endRingIndex
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (startOrientation
    :reader startOrientation
    :initarg :startOrientation
    :type cl:float
    :initform 0.0)
   (endOrientation
    :reader endOrientation
    :initarg :endOrientation
    :type cl:float
    :initform 0.0)
   (orientationDiff
    :reader orientationDiff
    :initarg :orientationDiff
    :type cl:float
    :initform 0.0)
   (segmentedCloudGroundFlag
    :reader segmentedCloudGroundFlag
    :initarg :segmentedCloudGroundFlag
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (segmentedCloudColInd
    :reader segmentedCloudColInd
    :initarg :segmentedCloudColInd
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (segmentedCloudRange
    :reader segmentedCloudRange
    :initarg :segmentedCloudRange
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass cloud_info (<cloud_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cloud_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cloud_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name AdaCt-msg:<cloud_info> is deprecated: use AdaCt-msg:cloud_info instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:header-val is deprecated.  Use AdaCt-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'timeStart-val :lambda-list '(m))
(cl:defmethod timeStart-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:timeStart-val is deprecated.  Use AdaCt-msg:timeStart instead.")
  (timeStart m))

(cl:ensure-generic-function 'timeEnd-val :lambda-list '(m))
(cl:defmethod timeEnd-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:timeEnd-val is deprecated.  Use AdaCt-msg:timeEnd instead.")
  (timeEnd m))

(cl:ensure-generic-function 'startRingIndex-val :lambda-list '(m))
(cl:defmethod startRingIndex-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:startRingIndex-val is deprecated.  Use AdaCt-msg:startRingIndex instead.")
  (startRingIndex m))

(cl:ensure-generic-function 'endRingIndex-val :lambda-list '(m))
(cl:defmethod endRingIndex-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:endRingIndex-val is deprecated.  Use AdaCt-msg:endRingIndex instead.")
  (endRingIndex m))

(cl:ensure-generic-function 'startOrientation-val :lambda-list '(m))
(cl:defmethod startOrientation-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:startOrientation-val is deprecated.  Use AdaCt-msg:startOrientation instead.")
  (startOrientation m))

(cl:ensure-generic-function 'endOrientation-val :lambda-list '(m))
(cl:defmethod endOrientation-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:endOrientation-val is deprecated.  Use AdaCt-msg:endOrientation instead.")
  (endOrientation m))

(cl:ensure-generic-function 'orientationDiff-val :lambda-list '(m))
(cl:defmethod orientationDiff-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:orientationDiff-val is deprecated.  Use AdaCt-msg:orientationDiff instead.")
  (orientationDiff m))

(cl:ensure-generic-function 'segmentedCloudGroundFlag-val :lambda-list '(m))
(cl:defmethod segmentedCloudGroundFlag-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:segmentedCloudGroundFlag-val is deprecated.  Use AdaCt-msg:segmentedCloudGroundFlag instead.")
  (segmentedCloudGroundFlag m))

(cl:ensure-generic-function 'segmentedCloudColInd-val :lambda-list '(m))
(cl:defmethod segmentedCloudColInd-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:segmentedCloudColInd-val is deprecated.  Use AdaCt-msg:segmentedCloudColInd instead.")
  (segmentedCloudColInd m))

(cl:ensure-generic-function 'segmentedCloudRange-val :lambda-list '(m))
(cl:defmethod segmentedCloudRange-val ((m <cloud_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader AdaCt-msg:segmentedCloudRange-val is deprecated.  Use AdaCt-msg:segmentedCloudRange instead.")
  (segmentedCloudRange m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cloud_info>) ostream)
  "Serializes a message object of type '<cloud_info>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeStart))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeEnd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'startRingIndex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'startRingIndex))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'endRingIndex))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'endRingIndex))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'startOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'endOrientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationDiff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segmentedCloudGroundFlag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'segmentedCloudGroundFlag))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segmentedCloudColInd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'segmentedCloudColInd))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segmentedCloudRange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'segmentedCloudRange))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cloud_info>) istream)
  "Deserializes a message object of type '<cloud_info>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeStart) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeEnd) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'startRingIndex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'startRingIndex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'endRingIndex) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'endRingIndex)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'startOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'endOrientation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationDiff) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segmentedCloudGroundFlag) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segmentedCloudGroundFlag)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segmentedCloudColInd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segmentedCloudColInd)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segmentedCloudRange) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segmentedCloudRange)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cloud_info>)))
  "Returns string type for a message object of type '<cloud_info>"
  "AdaCt/cloud_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cloud_info)))
  "Returns string type for a message object of type 'cloud_info"
  "AdaCt/cloud_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cloud_info>)))
  "Returns md5sum for a message object of type '<cloud_info>"
  "49d7e15d04cb07d4b59e4fdce8db7d1f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cloud_info)))
  "Returns md5sum for a message object of type 'cloud_info"
  "49d7e15d04cb07d4b59e4fdce8db7d1f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cloud_info>)))
  "Returns full string definition for message of type '<cloud_info>"
  (cl:format cl:nil "Header header~%~%float64 timeStart~%float64 timeEnd~%~%~%int32[] startRingIndex~%int32[] endRingIndex~%~%float32 startOrientation~%float32 endOrientation~%float32 orientationDiff~%~%bool[]    segmentedCloudGroundFlag # true - ground point, false - other points~%uint32[]  segmentedCloudColInd # point column index in range image~%float32[] segmentedCloudRange # point range~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cloud_info)))
  "Returns full string definition for message of type 'cloud_info"
  (cl:format cl:nil "Header header~%~%float64 timeStart~%float64 timeEnd~%~%~%int32[] startRingIndex~%int32[] endRingIndex~%~%float32 startOrientation~%float32 endOrientation~%float32 orientationDiff~%~%bool[]    segmentedCloudGroundFlag # true - ground point, false - other points~%uint32[]  segmentedCloudColInd # point column index in range image~%float32[] segmentedCloudRange # point range~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cloud_info>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'startRingIndex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'endRingIndex) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segmentedCloudGroundFlag) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segmentedCloudColInd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segmentedCloudRange) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cloud_info>))
  "Converts a ROS message object to a list"
  (cl:list 'cloud_info
    (cl:cons ':header (header msg))
    (cl:cons ':timeStart (timeStart msg))
    (cl:cons ':timeEnd (timeEnd msg))
    (cl:cons ':startRingIndex (startRingIndex msg))
    (cl:cons ':endRingIndex (endRingIndex msg))
    (cl:cons ':startOrientation (startOrientation msg))
    (cl:cons ':endOrientation (endOrientation msg))
    (cl:cons ':orientationDiff (orientationDiff msg))
    (cl:cons ':segmentedCloudGroundFlag (segmentedCloudGroundFlag msg))
    (cl:cons ':segmentedCloudColInd (segmentedCloudColInd msg))
    (cl:cons ':segmentedCloudRange (segmentedCloudRange msg))
))
