#!/usr/bin/env roseus

(load "eus-aria-webcommands.l")


(defun keyboard-callback (msg)
  (if (string-equal (send msg :data) "y")
      (setq stat 1)))

(ros::subscribe "/euslisp/keyboard" std_msgs::String #'keyboard-callback)

;; main

;;(setq initial-pose (ros::get-param "initial-pose"))


(defun initiate ()
  (let ()
    (set-torque 21 0)
    (unix::usleep (* 100 1000))
    (set-torque 22 0)
    (unix::usleep (* 100 1000))
    (set-position-neck 0.0 0.0 0.0)
    (set-position-torso 0.0 0.0 0.0)
    (set-position 1 0.0)
    (unix::usleep (* 100 1000))
    (set-position 2 0.0)
    (unix::usleep (* 100 1000))
    (set-position 3 0.0)
    (unix::usleep (* 100 1000))
    (set-position 4 0.0)
    (unix::usleep (* 100 1000))
    (set-position 5 0.0)
    (unix::usleep (* 100 1000))
    (set-position 6 0.0)
    (unix::usleep (* 100 1000))
    (set-position 7 0.0)
    (unix::usleep (* 100 1000))
    (set-position 8 0.0)
    (unix::usleep (* 100 1000))
    (set-position 9 0.0)
    (unix::usleep (* 100 1000))
    (set-position 10 0.0)
    (unix::usleep (* 100 1000))
    (set-position 11 0.0)
    (unix::usleep (* 100 1000))
    (set-position 12 0.0)
    (unix::usleep (* 100 1000))
    (set-position 13 0.0)
    (unix::usleep (* 100 1000))
    (set-position 14 0.0)
    (unix::usleep (* 100 1000))
    ))