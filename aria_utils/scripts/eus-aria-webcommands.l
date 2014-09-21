#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus "eus_aria_webcommands")

;; set mode globals
(defvar *non* 0)
(defvar *tor* 1)
(defvar *pos* 2)
(defvar *joint-size* 30)

;; set control vectors
(setq *control-mode* (make-array *joint-size*))
(fill *control-mode* *non*)
(setq *goal-position* (make-array *joint-size*))
(fill *goal-position* 0.0)
(setq *goal-torque* (make-array *joint-size*))
(fill *goal-torque* 0.0)

;; set sensor globals
(defvar *sensors* 5)
(defvar *sensor-torque* 0)
(defvar *sensor-position* 1)
(defvar *sensor-temperature* 2)
(defvar *sensor-velocity* 3)
(defvar *sensor-voltage* 4)

;; set sensor arrays
(setq *goal-position~* (make-array *joint-size*))
(fill *goal-position~* 0.0)
(setq *goal-torque~* (make-array *joint-size*))
(fill *goal-torque~* 0.0)
(setq *temperature~* (make-array *joint-size*))
(fill *temperature~* 0.0)
(setq *velocity~* (make-array *joint-size*))
(fill *velocity~* 0.0)
(setq *voltage~* (make-array *joint-size*))
(fill *voltage~* 0.0)

;; set ros publishers
(ros::advertise "/currentor_socket/request/mode_vector" std_msgs::Float32MultiArray)
(ros::advertise "/currentor_socket/request/position_vector" std_msgs::Float32MultiArray)
(ros::advertise "/currentor_socket/request/torque_vector" std_msgs::Float32MultiArray)
(ros::advertise "/ros2http/socket_listener/json_string" std_msgs::String)

;; set ros subscribers
(defun position-callback
  (msg)
  (setq *goal-position~* (send msg :data)))
(ros::subscribe "/currentor_socket/sensor_array/position" std_msgs::Float32MultiArray #'position-callback)
(defun torque-callback
  (msg)
  (setq *goal-torque~* (send msg :data)))
(ros::subscribe "/currentor_socket/sensor_array/torque" std_msgs::Float32MultiArray #'torque-callback)
(defun temperature-callback
  (msg)
  (setq *temperature~* (send msg :data)))
(ros::subscribe "/currentor_socket/sensor_array/temperature" std_msgs::Float32MultiArray #'temperature-callback)
(defun velocity-callback
  (msg)
  (setq *velocity~* (send msg :data)))
(ros::subscribe "/currentor_socket/sensor_array/velocity" std_msgs::Float32MultiArray #'velocity-callback)
(defun voltage-callback
  (msg)
  (setq *voltage~* (send msg :data)))
(ros::subscribe "/currentor_socket/sensor_array/voltage" std_msgs::Float32MultiArray #'voltage-callback)

;; -----------------------------------------------------

;; get functions
(defun get-value (id)
  (let (res)
    (ros::spin-once)
    (setq res (make-array *sensors*))
    (setf (aref res *sensor-torque*) (aref *goal-torque~* id))
    (setf (aref res *sensor-position*) (aref *goal-position~* id))
    (setf (aref res *sensor-temperature*) (aref *temperature~* id))
    (setf (aref res *sensor-velocity*) (aref velocity~* id))
    (setf (aref res *sensor-voltage*) (aref *voltage~* id))
    res))

(defun get-values ()
  (let (res)
    (ros::spin-once)
    (setq res (make-array *sensors*))
    (setf (aref res *sensor-torque*) *goal-torque~*)
    (setf (aref res *sensor-position*) *goal-position~*)
    (setf (aref res *sensor-temperature*) *temperature~*)
    (setf (aref res *sensor-velocity*) *velocity~*)
    (setf (aref res *sensor-voltage*) *voltage~*)
    res))

;; -----------------------------------------------------

;; control mode functions
(defun set-control-mode (id mode)
  (let (mode-msg)
    (setf (aref *control-mode* id) mode)
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
(defun send-zero ()
  (let (msg)
    (setq msg (instance std_msgs::String :init))
    (send msg :data (format nil "{~A:~A,~A:[0],~A:\"1\"}" "\"method\"" "\"sendZero\"" "\"params\"" "\"id\""))
    (ros::publish "/ros2http/socket_listener/json_string" msg))) 
  

(defun set-torque (joint torque)
  (let (res mode-msg torque-msg)
    (setq res (make-array 2))
    (setf (aref res 0) (setf (aref *control-mode* joint) *tor*))
    (setf (aref res 1) (setf (aref *goal-torque* joint) torque))
    (setq mode-msg (instance std_msgs::Float32MultiArray :init))
    (send mode-msg :data *control-mode*)
    (ros::publish "/currentor_socket/request/mode_vector" mode-msg)
    (unix::usleep (* 100 1000))
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
    (unix::usleep (* 100 1000))
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

;; set pid gain
(defun set-pid-gain (id p i d)
  (let (gain-msg)
    (setq gain-msg (instance std_msgs::String :init))
    (send gain-msg :data (format nil "{~A:~A,~A:[~A,~A,~A,~A],~A:\"1\"}" "\"method\"" "\"setPIDGain\"" "\"params\"" id p i d "\"id\""))
    (ros::publish "/ros2http/socket_listener/json_string" gain-msg))) 



;; -----------------------------------------------------

;; initialization
(setq *goal-position* (aref (get-values) *sensor-position*))
(setq *goal-torque* (aref (get-values) *sensor-torque*))

;; -----------------------------------------------------

;; other util functions
(defun set-position-neck (yaw roll pitch)
  (setf (aref *control-mode* 15) *pos*)
  (setf (aref *control-mode* 16) *pos*)
  (setf (aref *control-mode* 17) *pos*)
  (set-control-modes *control-mode*)
  (unix::usleep (* 100 1000))
  (setf (aref *goal-position* 15) yaw)
  (setf (aref *goal-position* 16) roll)
  (setf (aref *goal-position* 17) pitch)
  (set-positions *goal-position*))

(defun set-position-torso (yaw roll pitch)
  (setf (aref *control-mode* 18) *pos*)
  (setf (aref *control-mode* 19) *pos*)
  (setf (aref *control-mode* 20) *pos*)
  (set-control-modes *control-mode*)
  (unix::usleep (* 100 1000))
  (setf (aref *goal-position* 18) yaw)
  (setf (aref *goal-position* 19) roll)
  (setf (aref *goal-position* 20) pitch)
  (set-positions *goal-position*))