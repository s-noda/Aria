#!/usr/bin/env roseus

(ros::roseus "sasabody_commander")

(defvar *joint-cnt* 30)
(defvar *joint-vector* (instantiate float-vector *joint-cnt*))

(defun sensor-callback
  (msg &key (gain 9.8))
  (setf (aref *joint-vector* 21) (* gain (aref (send msg :data) 0)))
  (setf (aref *joint-vector* 22) (* -1 (aref *joint-vector* 21))))

(ros::advertise "/currentor_socket/request/torque_vector"
		std_msgs::float32multiarray)
(ros::subscribe "/invented_pendulum/command_vector"
		std_msgs::float32multiarray
		#'sensor-callback
		1)

(ros::rate 50)
(do-until-key
 (bench
  (if (not (ros::ok)) (return-from nil nil))
  (ros::sleep)
  (ros::spin-once)
  (ros::publish "/currentor_socket/request/torque_vector"
		(instance std_msgs::float32multiarray :init
			  :data *joint-vector*))
  ))
