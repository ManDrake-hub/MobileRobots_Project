;; Auto-generated. Do not edit!


(when (boundp 'navigation::Calibration)
  (if (not (find-package "NAVIGATION"))
    (make-package "NAVIGATION"))
  (shadow 'Calibration (find-package "NAVIGATION")))
(unless (find-package "NAVIGATION::CALIBRATION")
  (make-package "NAVIGATION::CALIBRATION"))
(unless (find-package "NAVIGATION::CALIBRATIONREQUEST")
  (make-package "NAVIGATION::CALIBRATIONREQUEST"))
(unless (find-package "NAVIGATION::CALIBRATIONRESPONSE")
  (make-package "NAVIGATION::CALIBRATIONRESPONSE"))

(in-package "ROS")



(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass navigation::CalibrationRequest
  :super ros::object
  :slots ())

(defmethod navigation::CalibrationRequest
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

(defclass navigation::CalibrationResponse
  :super ros::object
  :slots (_answer ))

(defmethod navigation::CalibrationResponse
  (:init
   (&key
    ((:answer __answer) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _answer __answer)
   self)
  (:answer
   (&rest __answer)
   (if (keywordp (car __answer))
       (send* _answer __answer)
     (progn
       (if __answer (setq _answer (car __answer)))
       _answer)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _answer
    (send _answer :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _answer
       (send _answer :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _answer
     (send _answer :deserialize buf ptr-) (incf ptr- (send _answer :serialization-length))
   ;;
   self)
  )

(defclass navigation::Calibration
  :super ros::object
  :slots ())

(setf (get navigation::Calibration :md5sum-) "a255970bffa18c4d56f14a3df609a6be")
(setf (get navigation::Calibration :datatype-) "navigation/Calibration")
(setf (get navigation::Calibration :request) navigation::CalibrationRequest)
(setf (get navigation::Calibration :response) navigation::CalibrationResponse)

(defmethod navigation::CalibrationRequest
  (:response () (instance navigation::CalibrationResponse :init)))

(setf (get navigation::CalibrationRequest :md5sum-) "a255970bffa18c4d56f14a3df609a6be")
(setf (get navigation::CalibrationRequest :datatype-) "navigation/CalibrationRequest")
(setf (get navigation::CalibrationRequest :definition-)
      "
---
std_msgs/String answer


================================================================================
MSG: std_msgs/String
string data
")

(setf (get navigation::CalibrationResponse :md5sum-) "a255970bffa18c4d56f14a3df609a6be")
(setf (get navigation::CalibrationResponse :datatype-) "navigation/CalibrationResponse")
(setf (get navigation::CalibrationResponse :definition-)
      "
---
std_msgs/String answer


================================================================================
MSG: std_msgs/String
string data
")



(provide :navigation/Calibration "a255970bffa18c4d56f14a3df609a6be")


