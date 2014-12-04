#!/usr/bin/env roseus

;; #-:jsk (if (functionp 'jsk) (jsk))
;; #-:rbrain (if (functionp 'rbrain) (rbrain))

(ros::roseus "ros_invented_pendulm_node")
(ros::load-ros-manifest "std_msgs")

(require "invented-pendulum.l")
(defvar *ip* (instance invented-pendulm :init))
;; (send *ip* :riccati-l)
(send *ip* :riccati-f)
(send *ip* :check-f)

(defvar *u* (send *ip* :u))
(defvar *x* (send *ip* :x))

;; (defvar *graph*)
;; (cond
;;  ((probe-file (ros::resolve-ros-path "package://euslib/demo/s-noda/util/graph-sample.l"))
;;   (require "package://euslib/demo/s-noda/util/graph-sample.l")
;;   (setq *graph*
;; 	(create-graph
;; 	 "invented-pendulum"
;; 	 :name-list (append (list "x" "q")
;; 			    (let* ((id 0))
;; 			      (map cons #'(lambda (v) (format nil "u~A" (incf id))) *u*)))
;; 	 ))
;;   (send *graph* :forcus-pos #F(0 0))
;;   ))

;; (defun update-graph
;;   (&key (rate 1))
;;   (cond
;;    (*graph*
;;     (add-data-to-graph
;;      (concatenate cons
;; 		  (list (aref (send *ip* :x^) 0)
;; 			(aref (send *ip* :x^) 1))
;; 		  *u*)
;;      :graph *graph*
;;      :time-step (* rate (send *ip* :dt))))))

(defun sensor-vector-callback
  (msg)
  (setq *x* (send msg :data))
  )

(defun torque-offset-callback
  (msg)
  (if (eq (length (send msg :data)) 2)
      (setf (aref *u* 0)
	    (+ (aref *u* 0)
	       (/ (- (aref (send msg :data) 1)
		     (aref (send msg :data) 0))
		  2.0)))))

(ros::subscribe "/invented_pendulum/sensor_vector"
		std_msgs::float32multiarray
		#'sensor-vector-callback
		1)
(ros::subscribe "/invented_pendulum/torque_offset_vector"
		std_msgs::float32multiarray
		#'torque-offset-callback
		1)

(ros::advertise "/invented_pendulum/command_vector"
		std_msgs::float32multiarray)
(ros::advertise "/invented_pendulum/observed_state_vector"
		std_msgs::float32multiarray)

(ros::rate 50)
(let* ((id 0))
  (do-until-key
   (bench
    (if (not (ros::ok)) (return-from nil nil))
    (ros::spin-once)
    (setq *u*
	  (send *ip* :update :u *u* :ys (transform (send *ip* :C) *x*)
		:x-callback #'(lambda (x)
				(v+
				 (map float-vector #'* #F(0 0 0 0) x)
				 (map float-vector #'* #F(0 1 0 1) *x*)))
		:x^-callback #'(lambda (x)
				 (v+
				  (map float-vector #'* #F(0 0 0 0) x)
				  (map float-vector #'* #F(0 1 0 1) *x*)))
		))
    (format t " x=~A/ u=~A  " (send *ip* :x^) *u*)
    ;; (cond
    ;;  ((> (incf id) 50)
    ;;   (setq id 0)
    ;;   (update-graph :rate 50)))
    (ros::publish "/invented_pendulum/command_vector"
		  (instance std_msgs::float32multiarray :init :data *u*))
    (ros::publish "/invented_pendulum/observed_state_vector"
		  (instance std_msgs::float32multiarray :init :data (send *ip* :x^)))
    (ros::sleep))))
