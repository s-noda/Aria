#!/usr/bin/env roseus

(require "package://aria_model/euslisp/aria-interface.l")

(aria-init)
(send-all (send *robot* :joint-list) :min-angle -180)
(send-all (send *robot* :joint-list) :max-angle +180)

(defun interpolator-callback
  (msg)
  (let* ((args (read-from-string (send msg :data)))
	 (av (car args))
	 (tm (cadr args))
	 (min-tm 50.0)
	 (cnt (/ tm min-tm))
	 rate mtm)
    (format t "[start-interpolation] ~A~%" (send msg :data))
    (setq mtm (instance mtimer :init))
    (dotimes (i (+ cnt 1))
      (setq rate (/ (* 1.0 i) cnt))
      (send *ri* :_angle-vector
	    (v+ (scale (- 1 rate) (send *robot* :angle-vector))
		(scale rate av)))
      (unix:usleep (round (* min-tm 1000))))
    (format t "   ~A~%" (send mtm :stop))
    ))

(ros::subscribe "/aria_interpolator/position"
		std_msgs::string
		#'interpolator-callback
		10)

(ros::rate 30)
(do-until-key
 (ros::sleep)
 (ros::spin-once)
 (send *robot* :angle-vector
       (send *ri* :state :potentio-vector :spin? nil))
 (send *robot* :newcoords
       (send *ri* :state :worldcoords  :spin? nil))
 (x::window-main-one)
 (send *viewer* :draw-objects))
