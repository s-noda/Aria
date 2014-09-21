#!/usr/bin/env roseus

(format t "~%~%[aria-simulator]~%")
(require "package://aria_model/euslisp/aria-interface.l")
(require "package://aria_model/euslisp/torque-util.l")

(aria-init :viewer? nil)
(ros::advertise "/aria_simulator/start/position/rarm"
		std_msgs::float32multiarray)
(ros::advertise "/aria_simulator/end/position/rarm"
		std_msgs::float32multiarray)
(ros::advertise "/aria_simulator/predict/position/rarm"
		std_msgs::float32multiarray)
(ros::advertise "/aria_simulator/predict/position/error/rarm"
		std_msgs::float32multiarray)

(defun get-vector
  (&key
   (all-links *all-links*)
   (command 'position)
   )
  (map float-vector
       #'(lambda (l)
	   (or (send (send l :joint) :get command) 0))
       all-links))

(defvar *all-links* (cdr (send *robot* :rarm :links)))

(defvar *torque-buf* (list (get-vector :command 'torque)))
(defvar *position-buf* (list (get-vector :command 'position)))
(defvar *velocity-buf* (list (get-vector :command 'velocity)))

(defvar *sim-timer*)
(defvar *sim-time-step* 0.033)

(ros::rate (round (/ 1 *sim-time-step*)))
(format t "   start !!~%")
(do-until-key
 (ros::sleep)
 (let* ((pb *position-buf*)
	(tb *torque-buf*)
	(vb *velocity-buf*)
	err
	q)
   (setq
    q
    (debug-calc-ddq-coeff-matrix-simulation
     :robot *robot*
     :all-links *all-links*
     :q (car (last *position-buf*))
     :dq (map float-vector #'deg2rad
	      (car (last *velocity-buf*)))
     :tau (reverse *torque-buf*)
     :animate? nil
     :sim-time-step *sim-time-step*
     :sim-time-max 0.33
     :init
     (progn
       (setq *position-buf* nil)
       (setq *velocity-buf* nil)
       (setq *torque-buf* nil)
       (setq *sim-timer* (instance mtimer :init))
       )
     :callback
     #'(lambda (&rest args)
	 (ros::spin-once)
	 (push (get-vector :command 'position) *position-buf*)
	 (push (get-vector :command 'velocity) *velocity-buf*)
	 (push (get-vector :command 'torque) *torque-buf*)
	 (let* ((tm (send *sim-timer* :stop)))
	   (cond
	    ((> tm *sim-time-step*)
	     (format t " --- overslept ~A > ~A~%"
		     tm *sim-time-step*))
	    (t
	     (unix:usleep
	      (round (* 1000 1000 (- *sim-time-step* tm)))))))
	 (send *sim-timer* :start))))
   (ros::publish "/aria_simulator/start/position/rarm"
		 (instance std_msgs::float32multiarray :init
			   :data (car (last pb))))
   (ros::publish "/aria_simulator/end/position/rarm"
		 (instance std_msgs::float32multiarray :init
			   :data (car pb)))
   (ros::publish "/aria_simulator/predict/position/rarm"
		 (instance std_msgs::float32multiarray :init
			   :data q))
   (ros::publish "/aria_simulator/predict/position/error/rarm"
		 (instance std_msgs::float32multiarray :init
			   :data (setq err (v- q (car pb)))))
   (map cons
	#'(lambda (err name thre)
	    (if (> (abs err) thre)
		(format t "  [~A] ~A > ~A~%" name err thre)))
	err
	(list :rarm-wrist-p :rarm-wrist-y :rarm-elbow-p :rarm-elbow-y :rarm-shoulder-r :rarm-shoulder-p :rarm-shoulder-y)
	'(15 5 5 5 3 3 3))
   ))
