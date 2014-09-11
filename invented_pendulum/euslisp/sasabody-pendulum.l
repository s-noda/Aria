#!/usr/bin/env roseus

(ros::roseus "sasabody_pendulum")

(require "euslib/demo/sasabuchi/sasabody/model/sasabody.l")

(defvar *robot* (sasabody))
(objects *robot*)
(send *irtviewer* :change-background #F(0.9 0.9 0.9))
(send *irtviewer* :draw-objects)

(send-all (send *robot* :joint-list) :min-angle -10000)
(send-all (send *robot* :joint-list) :max-angle 10000)

(send *robot* :put :imu-coords
      (let* ((bl (find-if #'(lambda (a) (string-equal (send a :name) "body_link"))
			  (send *robot* :links))))
	(make-cascoords
	 :name "imu_coords"
	 :coords
	 (send
	  (send
	   (send bl :copy-worldcoords)
	   :rotate (deg2rad -90) :x)
	  :rotate (deg2rad (* 2 90)) :z)
	 :parent bl)
	))

(defvar *joint-cnt* 30)
(defvar *joint-vector* (instantiate float-vector *joint-cnt*))
(defvar *joint-list*
  (mapcar
   #'(lambda (name)
       (or
	(find-if #'(lambda (j) (string-equal name (send j :name)))
		 (send *robot* :joint-list))
	(instance rotational-joint :init :child-link (make-coords)
		  :min -360 :max 360
		  )))
   (list
    "dummy"
    "arm_r_joint1" "arm_r_joint2" "arm_r_joint3" "arm_r_joint4"
    "arm_r_joint5" "arm_r_joint6" "arm_r_joint7" ;;"arm_l_hand1"
    "arm_l_joint1" "arm_l_joint2" "arm_l_joint3" "arm_l_joint4"
    "arm_l_joint5" "arm_l_joint6" "arm_l_joint7" ;;"arm_r_hand1"
    "neck1" "neck2" "neck3"
    "hip2" "hip3" "body"
    "dummy" "dummy"
    )))
(defvar *mirror-bin*
  (list
   0
   1 1 1 -1 -1 -1 -1
   1 1 1 -1 +1 -1 +1
   1 1 1
   1 0.5 -0.5
   1 1))

(defvar *position-updated* nil)
(defun position-callback
  (msg
   &key (draw? nil))
  (map cons
       #'(lambda (mir J pos)
	   (send j :joint-angle (* mir (rad2deg pos))))
       *mirror-bin*
       *joint-list*
       (send msg :data))
  (setq *position-updated* t)
  (cond
   (draw?
    (send *viewer* :draw-objects)
    (x::window-main-one)))
  )

(defvar *gyro* 0)
(defun imu-callback
  (msg
   &key
   (draw? t) (flush nil))
  (let* ((ori ;;(send (send (send msg :pose) :pose) :orientation))
	  (send msg :orientation))
	 (mat (quaternion2matrix
	       (float-vector (send ori :w) (send ori :x)
			     (send ori :y) (send ori :z))))
	 (tmp-coords (make-coords :rot mat))
	 (move-coords (copy-object (send (send *robot* :get :imu-coords) :worldcoords)))
	 )
    (setq *gyro*
	  (+ (* -0.5 (/ (* 2279 3.14) (* 180 (/ 32767 4.0)))
		(send (send msg :angular_velocity) :y))
	     (* 0.5 *gyro*)))
    (setq move-coords (send move-coords :transformation *robot*))
    (send tmp-coords :transform move-coords :local)
    (dotimes (i 3)
      ;; (send *robot* :newcoords (make-coords :rot mat)))
      (send *robot* :newcoords ;; (copy-object tmp-coords))
	    (copy-object (make-coords;; :pos (send tmp-coords :worldpos)
			  :rpy (map cons #'*
				    #F(0 1 1)
				    (car (rpy-angle (send tmp-coords :worldrot)))))))
      (send *robot* :worldcoords)
      ;; (send *robot* ;;(car (send *robot* :links))
      ;; 	    :transform
      ;; 	    (print
      ;; 	     (send (send body-link :worldcoords)
      ;; 		   :transformation
      ;; 		   (make-coords :rot mat)
      ;; 		   ))
      ;; 	    :local
      ;; 	    )
      )
    (cond
     (draw?
      (send *viewer* :draw-objects :flush flush)
      (send (make-coords :rot mat) :draw-on :flush flush
	    :color #f(1 0 0) :size 100 :width 100)
      ;; (send *viewer* :viewsurface :flush)
      ))
    ))

(ros::subscribe "/currentor_socket/sensor_array/position"
		std_msgs::float32multiarray
		#'position-callback
		1)
(ros::subscribe "/kduino/converted_imu" ;; "/imu_data"
                sensor_msgs::imu
		#'imu-callback
                1)

(ros::advertise "/invented_pendulum/sensor_vector"
		std_msgs::float32multiarray)


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
  (send *viewer* :viewsurface :flush)
  (x::window-main-one)
  ;; (print (calc-tilt))
  ;;(format t " ~A vs ~A~%"
  ;;(send (nth 0 (reverse *joint-list*)) :joint-angle)
  ;;(send (nth 1 (reverse *joint-list*)) :joint-angle))
  ;; (print (calc-forward))
  (ros::publish "/invented_pendulum/sensor_vector"
		(instance std_msgs::float32multiarray :init
			  :data (print (float-vector (calc-forward) (calc-tilt) 0 *gyro*))))
  ))
