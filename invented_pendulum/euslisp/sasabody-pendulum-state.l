#!/usr/bin/env roseus

(ros::roseus "sasabody_pendulum")

(defvar *state-vector* #F(0 0 0 0))

(defvar *wheel-vector* #F(0 0))
(defvar *position-vel* 0)
(defvar *position-timer*)
(defun position-callback
  (msg
   &key (draw? nil))
  (setf (aref *wheel-vector* 0) (aref (send msg :data) 22))
  (setf (aref *wheel-vector* 1) (aref (send msg :data) 21))
  ;;(setq *pos* (calc-forward))
  (let* ((x (calc-forward)))
    (if *position-timer*
	(setq *position-vel*
	      (deg2rad
	       (/ (- (aref *state-vector* 0) x) (send *position-timer* :stop)))))
    (setf (aref *state-vector* 0) x))
  (setq *position-timer* (instance mtimer :init))
  )

(defvar *imu-vel* 0)
(defvar *imu-timer*)
(defun imu-callback
  (msg
   &key
   (draw? t) (flush nil))
  (let* ((ori ;;(send (send (send msg :pose) :pose) :orientation))
	  (send msg :orientation))
	 (mat-raw (quaternion2matrix
		   (float-vector (send ori :w) (send ori :x)
				 (send ori :y) (send ori :z))))
	 (mat (send (make-coords
		     :rpy (map cons #'*
			       '(0 1 1)
			       (car (rpy-angle mat-raw))))
		    :worldrot))
	 )
    (setf (aref *state-vector* 1) (calc-tilt mat))
    (setf (aref *state-vector* 3)
	  (+ (* -0.9 (send (send msg :angular_velocity) :y))
	     (* 0.1 (aref *state-vector* 3))))
    (if *imu-timer*
	(setq *imu-vel*
	      (+ (aref *state-vector* 2)
		 (* (send *imu-timer* :stop) ;; -0.02
		    (send (send msg :linear_acceleration) :x)))))
    (setq *imu-timer* (instance mtimer :init))
    ))

(defvar *centroid* #F(0 0 1))
(defun centroid-callback (msg) (setq *centroid* (send msg :data)))

(defvar *wheel-zero* (copy-object *wheel-vector*))
(defvar *prev-wheel-angle-raw* (copy-object *wheel-vector*))
(defvar *prev-wheel-angle* (copy-object *wheel-vector*))
(defun calc-forward
  (&key (r 0.1))
  (let* ((now
	  (map float-vector
	       #'(lambda (prev-raw prev now)
		   (+
		    prev
		    (car (sort
			  (list (- now prev-raw)
				(+ (- now prev-raw) 360)
				(- (- now prev-raw) 360))
			  #'(lambda (a b) (< (abs a) (abs b)))))))
	       *prev-wheel-angle-raw*
	       *prev-wheel-angle*
	       (setq *prev-wheel-angle-raw* (copy-object *wheel-vector*)))))
    ;; (print now)
    (* r
       (deg2rad
	(v. #F(-0.5 0.5)
	    (map float-vector #'- (setq *prev-wheel-angle* now) *wheel-zero*))))))

(defun calc-tilt
  (rot)
  (let* ((x (transform rot #F(1 0 0)))
	 (v (normalize-vector
	     (v+
	      (map float-vector #'* #F(1 0 1)
		   (transform rot *centroid*))
	      #F(0 0 0.05))
	     )))
    ;;(format t ">>>>>>>>>>>> ")
    ;;(print
     (* (if (plusp (v. v #F(1 0 0))) -1 1)
	(acos (v. v #F(0 0 1))))))


(ros::subscribe "/currentor_socket/sensor_array/position"
		std_msgs::float32multiarray
		#'position-callback
		1)
(ros::subscribe "/kduino/converted_imu" ;; "/imu_data"
                sensor_msgs::imu
		#'imu-callback
                1)
(ros::subscribe "/sasabody_viewer/centroid" ;; "/imu_data"
                std_msgs::float32multiarray
		#'centroid-callback
                1)

(ros::advertise "/invented_pendulum/sensor_vector"
		std_msgs::float32multiarray)


;; (defun debug-callback (msg)
;;   ;; (format t " ~A vs ~A~%" (send msg :data) *state-vector*)
;;   ;; (format t "~A/~A = ~A~%"
;;   ;; 	  (aref (v- *state-vector* (send msg :data)) 1)
;;   ;; 	  (aref *state-vector* 1)
;;   ;; 	  (/ (aref (v- *state-vector* (send msg :data)) 1)
;;   ;; 	     (aref *state-vector* 1)
;;   ;; 	     ))
;;   (format t "~A - ~A/~A = ~A~%"
;; 	  (aref *state-vector* 0)
;; 	  (aref (send msg :data) 0)
;; 	  (aref *state-vector* 0)
;; 	  (/ 
;; 	     (aref *state-vector* 3)
;; 	     (aref (send msg :data) 3)
;; 	     ))
;;   )
;; (ros::subscribe "/invented_pendulum/sensor_vector"
;; 		std_msgs::float32multiarray
;; 		#'debug-callback
;; 		1)

(ros::rate 100)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::sleep)
 (ros::spin-once)
 (let* ((vel-candidates (flatten (list *imu-vel* *position-vel*))))
   (if vel-candidates
       (setf (aref *state-vector* 2)
	     (/ (apply #'+ (append vel-candidates '(0 0)))
		(length vel-candidates)))))
 (format t " velcan ~A vs ~A" *imu-vel* *position-vel*)
 (ros::publish "/invented_pendulum/sensor_vector"
	       (instance std_msgs::float32multiarray :init
			 :data (print *state-vector*)))
 )
