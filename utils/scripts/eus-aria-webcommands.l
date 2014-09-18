#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")

;; ros setup
(ros::roseus "eus_aria_webcommands")
(ros::advertise "/currentor_socket/request/mode_vector" std_msgs::Float32MultiArray)
(ros::advertise "/currentor_socket/request/position_vector" std_msgs::Float32MultiArray)
(ros::advertise "/currentor_socket/request/torque_vector" std_msgs::Float32MultiArray)


;; set mode globals
(defvar *non* 0)
(defvar *tor* 1)
(defvar *pos* 2)
(defvar *joint-size* 30)

;; set eus-vector
(setq *control-mode* (make-array *joint-size*))
(fill *control-mode* *non*)
(setq *goal-position* (make-array *joint-size*))
(fill *goal-position* 0.0)
(setq *goal-torque* (make-array *joint-size*))
(fill *goal-torque* 0.0)

(ros::subscribe "/currentor_socket/sensor_array/position" std_msgs::Float32MultiArray
		#'(lambda (msg) 
		    (setq *goal-position* (send msg :data))
		    (ros::unsubscribe "/currenter_socket/sensor_array/position")))
(ros::spin-once)


;; -----------------------------------------------------

;; control mode functions
(defun set-control-mode (id mode)
  (let (mode-msg)
    (setq (aref *control-mode* id) mode)
    (setq mode-msg (instance std_msgs::Float32MultiArray :init))
    (send mode-msg :data *control-mode*)
    (ros::publish "/currentor_socket/request/mode_vector" mode-msg)))

(defun set-control-modes (modes)
  (let (mode-msg)
    (if (eq (length modes) *joint-size*)
	(progn (setq *control-mode* modes)
	       (setq mode-msg (instance std_msgs::Float32MultiArray :init))
	       (send mode-msg :data *control-mode*)
	       (ros::publish "/currentor_socket/request/mode_vector" mode-msg))
      (format nil "array size must be ~A" *joint-size*))))

;; send torque functions
(defun set-torque (joint torque)
  (let (res mode-msg torque-msg)
    (setq res (make-array 2))
    (setf (aref res 0) (setf (aref *control-mode* joint) *tor*))
    (setf (aref res 1) (setf (aref *goal-torque* joint) torque))
    (setq mode-msg (instance std_msgs::Float32MultiArray :init))
    (send mode-msg :data *control-mode*)
    (ros::publish "/currentor_socket/request/mode_vector" mode-msg)
    (setq torque-msg (instance std_msgs::Float32MultiArray :init))
    (send torque-msg :data *goal-torque*)
    (ros::publish "/currentor_socket/request/torque_vector" torque-msg)
    res))

(defun set-torques (torques)
  (let (torque-msg)
    (if (eq (length torques) *joint-size*)
	(progn (setq *goal-torque* torques)
	       (setq torque-msg (instance std_msgs::Float32MultiArray :init))
	       (send torque-msg :data *goal-torque*)
	       (ros::publish "/currentor_socket/request/torque_vector" torque-msg))
      (format nil "array size must be ~A" *joint-size*))))

;; send position functions
(defun set-position (joint angle)
  (let (res mode-msg pos-msg)
    (setq res (make-array 2))
    (setf (aref res 0) (setf (aref *control-mode* joint) *pos*))
    (setf (aref res 1) (setf (aref *goal-position* joint) angle))
    (setq mode-msg (instance std_msgs::Float32MultiArray :init))
    (send mode-msg :data *control-mode*)
    (ros::publish "/currentor_socket/request/mode_vector" mode-msg)
    (setq pos-msg (instance std_msgs::Float32MultiArray :init))
    (send pos-msg :data *goal-position*)
    (ros::publish "/currentor_socket/request/position_vector" pos-msg)
    res))

(defun set-positions (positions)
  (let (pos-msg)
    (if (eq (length positions) *joint-size*)
	(progn (setq *goal-position* positions)
	       (setq pos-msg (instance std_msgs::Float32MultiArray :init))
	       (send pos-msg :data *goal-position*)
	       (ros::publish "/currentor_socket/request/position_vector" pos-msg))
      (format nil "array size must be ~A" *joint-size*))))


;; -----------------------------------------------------

;; other util functions
(defun set-position-neck (yaw roll pitch)
  (setf (aref *control-mode* 15) *pos*)
  (setf (aref *control-mode* 16) *pos*)
  (setf (aref *control-mode* 17) *pos*)
  (set-control-modes *control-mode*)
  (unix::sleep 0.1)
  (setf (aref *goal-position* 15) yaw)
  (setf (aref *goal-position* 16) roll)
  (setf (aref *goal-position* 17) pitch)
  (set-positions *goal-position*))

(defun set-position-torso (yaw roll pitch)
  (setf (aref *control-mode* 18) *pos*)
  (setf (aref *control-mode* 19) *pos*)
  (setf (aref *control-mode* 20) *pos*)
  (set-control-modes *control-mode*)
  (unix::sleep 0.1)
  (setf (aref *goal-position* 18) yaw)
  (setf (aref *goal-position* 19) roll)
  (setf (aref *goal-position* 20) pitch)
  (set-positions *goal-position*))