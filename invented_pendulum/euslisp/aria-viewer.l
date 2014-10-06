#!/usr/bin/env roseus

(ros::roseus "aria_viewer")

(require "package://aria_model/aria.l")
(require "package://aria_model/euslisp/aria-interface.l")

(defvar *robot*)

(aria-init :viewer? nil)
(objects *robot*)
(send *irtviewer* :change-background #F(0.9 0.9 0.9))
(send *irtviewer* :draw-objects)
(send *irtviewer* :title "ARIA CENTROID VIEWER")

(send-all (send *robot* :joint-list) :min-angle -10000)
(send-all (send *robot* :joint-list) :max-angle 10000)

(ros::advertise "/aria_viewer/centroid"
                std_msgs::float32multiarray
                1)

(defun calc-inertia-tensor-from-coords ;; RTIR + m[cx]T[cx]
  (c
   &key
   (link-list (send *robot* :links))
   (meter? t))
  (reduce
   #'m+
   (mapcar
    #'(lambda (l)
        (let* ((p (outer-product-matrix (v- (send l :worldpos)
                                            (send c :worldpos))))
               (r (m* (transpose (send c :worldrot))
                      (send l :worldrot)))
               (i (send l :inertia-tensor))
               (m (send l :weight))
               (ret
                (m+ (m* (transpose r) (m* i r))
                    (scale-matrix m (m* (transpose p) p)))))
          (if meter? (scale-matrix 1e-9 ret) ret)))
    link-list)))

(ros::rate 100)
(do-until-key
 (bench
  (if (not (ros::ok)) (return-from nil nil))
  (ros::sleep)
  (ros::spin-once)
  (send *robot* :angle-vector
	(send *ri* :state :potentio-vector :spin? nil))
  (send *robot* :newcoords
	(send *ri* :state :worldcoords :spin? nil))
  (send *viewer* :draw-objects :flush nil)
  ;;(send (make-coords :rot mat) :draw-on :flush nil
  ;;:color #f(1 0 0) :size 100 :width 100)
  (send (send *robot* :centroid) :draw-on :flush nil
	:color #f(0 1 0) :size 100 :width 100)
  (send *viewer* :viewsurface :flush)
  (x::window-main-one)
  (ros::publish "/aria_viewer/centroid"
		(instance std_msgs::float32multiarray :init
			  :data
			  (transform
			   (transpose
			    (send
			     (send (send *robot* :get :imu-coords)
				   :worldcoords) :worldrot))
			   (send *robot* :centroid))))
  ))


#|


;; (ros::advertise "/invented_pendulum/sensor_vector"
;; 		std_msgs::float32multiarray)


;; (defun calc-tilt
;;   nil
;;   (let* ((x (transform (send *robot* :worldrot) #F(1 0 0)))
;; 	 (z ;;(transform (send *robot* :worldrot) #F(0 0 1)))
;; 	  (normalize-vector (map float-vector #'* #F(1 0 1) (send *robot* :centroid))))
;; 	 )
;;     (*
;;      (if (plusp (v. (v* x #F(1 0 0)) (v* z #F(1 0 0)))) -1 1)
;;      (acos (v. z #F(0 0 1))))))

(defun calc-tilt
  nil
  (let* ((x (transform (send *robot* :worldrot) #F(1 0 0)))
	 (z ;;(transform (send *robot* :worldrot) #F(0 0 1)))
	  (normalize-vector (map float-vector #'* #F(1 0 1) (send *robot* :centroid))))
	 )
    (*
     (if (plusp (aref z 0)) 1 -1)
     (acos (v. z #F(0 0 1))))))

(defvar *wheel-zero* (send-all (subseq (reverse *joint-list*) 0 2) :joint-angle))
(defvar *prev-wheel-angle-raw*
  (send-all (subseq (reverse *joint-list*) 0 2) :joint-angle))
(defvar *prev-wheel-angle*
  (send-all (subseq (reverse *joint-list*) 0 2) :joint-angle))
(defun calc-forward
  (&key (r 0.1))
  (let* ((now
	  (mapcar #'(lambda (prev-raw prev now)
		      (+
		       prev
		       (car (sort
			     (list (- now prev-raw)
				   (+ (- now prev-raw) 360)
				   (- (- now prev-raw) 360))
			     #'(lambda (a b) (< (abs a) (abs b)))))))
		  *prev-wheel-angle-raw*
		  *prev-wheel-angle*
		  (setq *prev-wheel-angle-raw*
			(send-all (subseq (reverse *joint-list*) 0 2) :joint-angle)))))
    ;; (print now)
    (* r
       (deg2rad
	(v. #F(-0.5 0.5)
	    (map float-vector #'- (setq *prev-wheel-angle* now) *wheel-zero*))))))

  ;; (print (calc-tilt))
  ;;(format t " ~A vs ~A~%"
  ;;(send (nth 0 (reverse *joint-list*)) :joint-angle)
  ;;(send (nth 1 (reverse *joint-list*)) :joint-angle))
  ;; (print (calc-forward))
  (ros::publish "/invented_pendulum/sensor_vector"
		(instance std_msgs::float32multiarray :init
			  :data (print (float-vector (calc-forward) (calc-tilt) 0 *gyro*))))
