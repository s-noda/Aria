#!/usr/bin/env roseus

(load "eus-aria-webcommands.l")

(setq stat 0)

;; subscribers for stat 0

(defun sensor-callback (msg)
  (let (err-zero-pose err-list)
    (setq err-zero-pose 0.1)
    (setq err-list nil)
    (dotimes (i (length (send msg :data)))
      (let (val)
	(setq val (elt (send msg :data) i))
	(if (> val err-zero-pose)
	    (setq err-list (append err-list (list val))))))
    (format nil "~A" (err-list))))

(ros::subscribe "/currentor_socket/sensor_array/position" std_msgs/Float32MultiArray #'sensor-callback)

(defun keyboard-callback (msg)
   (if (string-equal (send msg :data) "y")
       (setq stat 1)))

(ros::subscribe "/euslisp/keyboard" std_msgs/String #'keyboard-callback)

;; main

(setq initial-pose (ros::get-param "initial-pose"))
(ros::rate 100)

(while (ros::ok)
  ;; checking joints with wrong initiate value
  (if (eq stat 0)
      (ros::spin-once))
  ;; sending initial pose
  (if (eq stat 1)
      (progn
	(let ()
	  (send-pos-neck (aref initial-pose 15) (aref initial-pose 16) (aref initial-pose 17))
	  (sleep 1)
	  (send-pos-torso (aref initial-pose 18) (aref initial-pose 19) (aref initial-pose 20))
	  (sleep 1)
	  (send-pos-rarm (aref initial-pose 1) (aref initial-pose 2) (aref initial-pose 3) (aref initial-pose 4) (aref initial-pose 5) (aref initial-pose 6) (aref initial-pose 7))
	  (sleep 1)
	  (send-pos-larm (aref initial-pose 8) (aref initial-pose 9) (aref initial-pose 10) (aref initial-pose 11) (aref initial-pose 12) (aref initial-pose 13) (aref initial-pose 14))
	  (send-torque 21 0)
	  (send-torque 22 0)
	  )))
  (ros::sleep))