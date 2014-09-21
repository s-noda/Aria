#!/usr/bin/env roseus

(load "eus-aria-webcommands.l")

(setq stat 0)

(defun keyboard-callback (msg)
  (if (string-equal (send msg :data) "y")
      (setq stat 1)))

(ros::subscribe "/euslisp/keyboard" std_msgs::String #'keyboard-callback)

;; main

;;(setq initial-pose (ros::get-param "initial-pose"))


(ros::roseus-add-msgs "jsk_pcl_ros")




(defun box-callback (msg)
  ())

(ros::subscribe "/cluster_decomposer_final/boxes" jsk_pcl_ros::BoundingBoxArray #'box-callback)

(defun plane-callback (msg)
  ())

(ros::subscribe "/multi_plane_estimate/output_polygon" jsk_pcl_ros::PolygonArray #'plane-callback)


(defun initiate ()
  (let ()
    (print (format nil "is encoder okay?"))
    (dotimes (i 10000000)
      (ros::spin-once))
    (if (eq stat 0)
	(return-from initiate (format nil "run eus-keyboard and type y in adifferent terminal if encoder is okay")))
    (print (format nil "starting initation"))
    (set-torque 21 0)
    (unix::usleep (* 100 1000))
    (set-torque 22 0)
    (unix::usleep (* 100 1000))
    (set-position-neck 0.0 0.0 0.0)
    (set-position-torso 0.0 0.0 0.0)
    (unix::usleep (* 1000 1000))
    (set-position 1 0.0)
    (unix::usleep (* 100 1000))
    (set-position 2 0.0)
    (unix::usleep (* 100 1000))
    (set-position 3 0.0)
    (unix::usleep (* 100 1000))
    (set-position 4 0.0)
    (unix::usleep (* 100 1000))
    (set-position 5 1.57)
    (unix::usleep (* 100 1000))
    (set-position 6 0.0)
    (unix::usleep (* 100 1000))
    (set-position 7 0.0)
    (unix::usleep (* 1000 1000))
    (set-position 8 0.0)
    (unix::usleep (* 100 1000))
    (set-position 9 0.0)
    (unix::usleep (* 100 1000))
    (set-position 10 0.0)
    (unix::usleep (* 100 1000))
    (set-position 11 0.0)
    (unix::usleep (* 100 1000))
    (set-position 12 -1.57)
    (unix::usleep (* 100 1000))
    (set-position 13 0.0)
    (unix::usleep (* 100 1000))
    (set-position 14 0.0)
    (unix::usleep (* 100 1000))
    ))