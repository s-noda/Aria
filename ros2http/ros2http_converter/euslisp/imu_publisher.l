#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "nav_msgs")

(ros::roseus "imu_publisher")

(defvar *linear_acceleration-gain*)
(defvar *imu-coords* (make-coords))
(defvar *gyro-coords* (make-coords))
(defvar *comb-coords* (make-coords))
(defvar *last-time* (send (ros::time) :now))

(defun imu-callback
  (msg)
  (if (not *linear_acceleration-gain*)
      (setq *linear_acceleration-gain*
	    (/ 9.8 (norm (subseq (send msg :data) 0 3)))))
  (let* ((gs *linear_acceleration-gain*)
	 (now (send (ros::time) :now))
	 (gv (normalize-vector (subseq (send msg :data) 0 3)))
	 )
    (setq *imu-coords*
	  (make-coords
	   :pos
	   (v+
	    (scale (expt (- (send now :to-sec) (send *last-time* :to-sec)) 2)
		   (subseq (send msg :data) 0 3))
	    (copy-object (send *comb-coords* :worldpos)))
	   :rot
	   (matrix-exponent
	    (normalize-vector (v* #F(0 0 1) gv))
	    (acos (v. gv #F(0 0 1))))))
    (setq *gyro-coords*
	  (make-coords
	   :pos (copy-object (send *imu-coords* :worldpos))
	   :rot (m*
		 (matrix-exponent
		  (scale
		   (- (send now :to-sec) (send *last-time* :to-sec))
		   (subseq (send msg :data) 3 6)))
		 (send *comb-coords* :worldrot))))
    (setq *comb-coords* (midcoords 0.1 *imu-coords* *gyro-coords*))
    (ros::publish
     "/imu_data"
     (instance sensor_msgs::imu
	       :init
	       :header
	       (instance std_msgs::header :init
			 :stamp now
			 :frame_id ;;"/origin")
			 "imu_sensor")
	       :orientation
	       (let ((q (matrix2quaternion (send *comb-coords* :worldrot))))
		 (instance geometry_msgs::quaternion
			   :init
			   :x (aref q 1)
			   :y (aref q 2)
			   :z (aref q 3)
			   :w (aref q 0)))
	       :linear_acceleration
	       (instance geometry_msgs::Vector3
			 :init
			 :x (* gs (aref (send msg :data) 0))
			 :y (* gs (aref (send msg :data) 1))
			 :z (* gs (aref (send msg :data) 2)))
	       :angular_velocity
	       (instance geometry_msgs::Vector3
			 :init
			 :x (aref (send msg :data) 3)
			 :y (aref (send msg :data) 4)
			 :z (aref (send msg :data) 5))
	       :orientation_covariance
	       (scale -1 (send (unit-matrix) :get-val 'entity))
	       :linear_acceleration_covariance
	       (scale +1 (send (unit-matrix) :get-val 'entity))
	       :angular_velocity_covariance
	       (scale +1 (send (unit-matrix) :get-val 'entity))
	       ))
    (setq *last-time* now)
    ))

(defun dummy-odom-pub
  nil
  (let* ((q (matrix2quaternion (send *comb-coords* :worldrot)))
	 (p (send *comb-coords* :worldpos)))
    (ros::publish
     "/odom"
     (instance nav_msgs::odometry
	       :init
	       :header
	       (instance std_msgs::header :init
			 :stamp
			 (send (ros::time) :now)
			 :frame_id "odom")
	       :pose
	       (instance geometry_msgs::posewithcovariance
			 :init
			 :pose
			 (instance geometry_msgs::pose :init
				   :position
				   (instance geometry_msgs::point :init
					     :x (aref p 0)
					     :y (aref p 1)
					     :z (aref p 2))
				   :orientation
				   (instance geometry_msgs::quaternion :init
					     :x (aref q 1)
					     :y (aref q 2)
					     :z (aref q 3)
					     :w (aref q 0)))
			 :covariance
			 (scale +1e-4 (send (unit-matrix 6) :get-val 'entity))
			 )
	       :twist
	       (instance geometry_msgs::twistwithcovariance
			 :init
			 :covariance
			 (scale +1e-4 (send (unit-matrix 6) :get-val 'entity))
			 )
	       )
     )))

(ros::subscribe "/currentor_socket/sensor_array/imu"
		std_msgs::float32multiarray
		#'imu-callback
		1)
(ros::advertise "/imu_data"
		sensor_msgs::imu
		1)
(ros::advertise "/odom"
		nav_msgs::odometry
		1)

(ros::rate 100)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 (dummy-odom-pub)
 ;; (imu-callback (instance std_msgs::float32multiarray
 ;; 			 :init
 ;; 			 :data
 ;; 			 (concatenate
 ;; 			  float-vector
 ;; 			  (v+
 ;; 			   (random-vector 1.0)
 ;; 			   #F(0 0 9.8))
 ;; 			  (random-vector 1.0)
 ;; 			  )))
 )
