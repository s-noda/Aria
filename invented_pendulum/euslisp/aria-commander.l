#!/usr/bin/env roseus

(ros::roseus "aria_commander")

(defvar *joint-cnt* 30)
(defvar *joint-vector* (instantiate float-vector *joint-cnt*))
(defvar *torque-vector* (float-vector 0 0))

(defun sensor-callback
  (msg &key (gain (* 0.5 9.8)))
  (setf (aref *joint-vector* 22) (* gain (aref (send msg :data) 0)))
  (setf (aref *joint-vector* 21) (* -1 (aref *joint-vector* 22))))

(defun torque-offset-callback
  (msg &key (gain 9.8))
  (if (not (eq (length (send msg :data)) 2))
      (return-from torque-offset-callback nil))
  (setq *torque-vector* (scale gain (send msg :data)))
  )

(ros::advertise "/currentor_socket/request/torque_vector"
		std_msgs::float32multiarray)
(ros::advertise "/currentor_socket/request/wheel_torque_vector"
		std_msgs::float32multiarray)
(ros::subscribe "/invented_pendulum/command_vector"
		std_msgs::float32multiarray
		#'sensor-callback
		1)
(ros::subscribe "/invented_pendulum/torque_offset_vector"
		std_msgs::float32multiarray
		#'torque-offset-callback
		1)

(defun send-zero
  nil
  (fill *joint-vector* 0)
  (ros::publish "/currentor_socket/request/torque_vector"
		(instance std_msgs::float32multiarray :init
			  :data *joint-vector*)))

(ros::rate 50)
(do-until-key
 (bench
  (if (not (ros::ok)) (return-from nil nil))
  (setq *torque-vector* (scale 0.8 *torque-vector*))
  (ros::sleep)
  (ros::spin-once)
  (setf (aref *torque-vector* 0)
	(+ (aref *torque-vector* 0) (aref *joint-vector* 21)))
  (setf (aref *torque-vector* 1)
	(+ (aref *torque-vector* 1) (aref *joint-vector* 22)))
  ;; (ros::publish "/currentor_socket/request/torque_vector"
  ;; 		(instance std_msgs::float32multiarray :init
  ;; 			  :data *joint-vector*))
  (ros::publish "/currentor_socket/request/wheel_torque_vector"
   		(instance std_msgs::float32multiarray :init
			  :data *torque-vector*))
  ))
