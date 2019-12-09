; Auto-generated. Do not edit!


(cl:in-package lab4_cam-srv)


;//! \htmlinclude ImageSrv-request.msg.html

(cl:defclass <ImageSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ImageSrv-request (<ImageSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab4_cam-srv:<ImageSrv-request> is deprecated: use lab4_cam-srv:ImageSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageSrv-request>) ostream)
  "Serializes a message object of type '<ImageSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageSrv-request>) istream)
  "Deserializes a message object of type '<ImageSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageSrv-request>)))
  "Returns string type for a service object of type '<ImageSrv-request>"
  "lab4_cam/ImageSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageSrv-request)))
  "Returns string type for a service object of type 'ImageSrv-request"
  "lab4_cam/ImageSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageSrv-request>)))
  "Returns md5sum for a message object of type '<ImageSrv-request>"
  "ba55116f263d40ea8759822097ad63d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageSrv-request)))
  "Returns md5sum for a message object of type 'ImageSrv-request"
  "ba55116f263d40ea8759822097ad63d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageSrv-request>)))
  "Returns full string definition for message of type '<ImageSrv-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageSrv-request)))
  "Returns full string definition for message of type 'ImageSrv-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageSrv-request
))
;//! \htmlinclude ImageSrv-response.msg.html

(cl:defclass <ImageSrv-response> (roslisp-msg-protocol:ros-message)
  ((image_data
    :reader image_data
    :initarg :image_data
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass ImageSrv-response (<ImageSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab4_cam-srv:<ImageSrv-response> is deprecated: use lab4_cam-srv:ImageSrv-response instead.")))

(cl:ensure-generic-function 'image_data-val :lambda-list '(m))
(cl:defmethod image_data-val ((m <ImageSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab4_cam-srv:image_data-val is deprecated.  Use lab4_cam-srv:image_data instead.")
  (image_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageSrv-response>) ostream)
  "Serializes a message object of type '<ImageSrv-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageSrv-response>) istream)
  "Deserializes a message object of type '<ImageSrv-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageSrv-response>)))
  "Returns string type for a service object of type '<ImageSrv-response>"
  "lab4_cam/ImageSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageSrv-response)))
  "Returns string type for a service object of type 'ImageSrv-response"
  "lab4_cam/ImageSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageSrv-response>)))
  "Returns md5sum for a message object of type '<ImageSrv-response>"
  "ba55116f263d40ea8759822097ad63d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageSrv-response)))
  "Returns md5sum for a message object of type 'ImageSrv-response"
  "ba55116f263d40ea8759822097ad63d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageSrv-response>)))
  "Returns full string definition for message of type '<ImageSrv-response>"
  (cl:format cl:nil "~%sensor_msgs/Image image_data~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageSrv-response)))
  "Returns full string definition for message of type 'ImageSrv-response"
  (cl:format cl:nil "~%sensor_msgs/Image image_data~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageSrv-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageSrv-response
    (cl:cons ':image_data (image_data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImageSrv)))
  'ImageSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImageSrv)))
  'ImageSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageSrv)))
  "Returns string type for a service object of type '<ImageSrv>"
  "lab4_cam/ImageSrv")