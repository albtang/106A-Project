;; Auto-generated. Do not edit!


(when (boundp 'lab4_cam::ImageSrv)
  (if (not (find-package "LAB4_CAM"))
    (make-package "LAB4_CAM"))
  (shadow 'ImageSrv (find-package "LAB4_CAM")))
(unless (find-package "LAB4_CAM::IMAGESRV")
  (make-package "LAB4_CAM::IMAGESRV"))
(unless (find-package "LAB4_CAM::IMAGESRVREQUEST")
  (make-package "LAB4_CAM::IMAGESRVREQUEST"))
(unless (find-package "LAB4_CAM::IMAGESRVRESPONSE")
  (make-package "LAB4_CAM::IMAGESRVRESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass lab4_cam::ImageSrvRequest
  :super ros::object
  :slots ())

(defmethod lab4_cam::ImageSrvRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass lab4_cam::ImageSrvResponse
  :super ros::object
  :slots (_image_data ))

(defmethod lab4_cam::ImageSrvResponse
  (:init
   (&key
    ((:image_data __image_data) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image_data __image_data)
   self)
  (:image_data
   (&rest __image_data)
   (if (keywordp (car __image_data))
       (send* _image_data __image_data)
     (progn
       (if __image_data (setq _image_data (car __image_data)))
       _image_data)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image_data
    (send _image_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image_data
       (send _image_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image_data
     (send _image_data :deserialize buf ptr-) (incf ptr- (send _image_data :serialization-length))
   ;;
   self)
  )

(defclass lab4_cam::ImageSrv
  :super ros::object
  :slots ())

(setf (get lab4_cam::ImageSrv :md5sum-) "ba55116f263d40ea8759822097ad63d4")
(setf (get lab4_cam::ImageSrv :datatype-) "lab4_cam/ImageSrv")
(setf (get lab4_cam::ImageSrv :request) lab4_cam::ImageSrvRequest)
(setf (get lab4_cam::ImageSrv :response) lab4_cam::ImageSrvResponse)

(defmethod lab4_cam::ImageSrvRequest
  (:response () (instance lab4_cam::ImageSrvResponse :init)))

(setf (get lab4_cam::ImageSrvRequest :md5sum-) "ba55116f263d40ea8759822097ad63d4")
(setf (get lab4_cam::ImageSrvRequest :datatype-) "lab4_cam/ImageSrvRequest")
(setf (get lab4_cam::ImageSrvRequest :definition-)
      "
---

sensor_msgs/Image image_data


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

(setf (get lab4_cam::ImageSrvResponse :md5sum-) "ba55116f263d40ea8759822097ad63d4")
(setf (get lab4_cam::ImageSrvResponse :datatype-) "lab4_cam/ImageSrvResponse")
(setf (get lab4_cam::ImageSrvResponse :definition-)
      "
---

sensor_msgs/Image image_data


================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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



(provide :lab4_cam/ImageSrv "ba55116f263d40ea8759822097ad63d4")


