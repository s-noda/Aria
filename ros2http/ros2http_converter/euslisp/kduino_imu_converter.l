#!/usr/bin/env roseus

(ros::load-ros-manifest "kduino")
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "kduino_imu_converter")

(defun kduino_imu_converter
  (msg)
  (let* ((av (map float-vector #'deg2rad (send msg :angle)))
	 (acc (send msg :accData))
	 (gyr (send msg :gyroData))
	 (q (matrix2quaternion (send (make-coords :rpy (reverse av)) :worldrot)))
	 )
    (ros::publish
     "kduino/converted_imu"
     (instance sensor_msgs::imu :init
	       :orientation
	       (instance geometry_msgs::quaternion :init
			 :w (aref q 0)
			 :x (aref q 1)
			 :y (aref q 2)
			 :z (aref q 3))
	       :angular_velocity
	       (instance geometry_msgs::vector3 :init
			 :x (aref acc 0)
			 :y (aref acc 1)
			 :z (aref acc 2))
	       :linear_acceleration
	       (instance geometry_msgs::vector3 :init
			 :x (aref gyr 0)
			 :y (aref gyr 1)
			 :z (aref gyr 2))))
    ))

(ros::subscribe "kduino/imu"
		kduino::KduinoImu
		#'kduino_imu_converter
		1)
(ros::advertise "kduino/converted_imu"
		sensor_msgs::imu
		1)

(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 )