#!/usr/bin/env roseus

(require "sample_json_publisher.l")
(require "euslib/demo/sasabuchi/sasabody/model/sasabody.l")

(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::unsubscribe "/ros2http/socket_listener/reponse")

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
	(instance rotational-joint :init :child-link (make-coords))))
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
   0 0))

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
    (setq move-coords (send move-coords :transformation *robot*))
    (send tmp-coords :transform move-coords :local)
    (dotimes (i 3)
      ;; (send *robot* :newcoords (make-coords :rot mat)))
      (send *robot* :newcoords (copy-object tmp-coords))
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

(defun torque-command
  (tv)
  (dotimes (i (length tv))
    (setf (aref *joint-vector* i) (elt tv i)))
  (ros::publish "/currentor_socket/request/torque_vector"
  		(instance std_msgs::float32multiarray
  			  :init
  			  :data *joint-vector*))
  )

;; (defun torque-command (tv) nil)

(ros::advertise "/currentor_socket/request/torque_vector"
		std_msgs::float32multiarray)
(ros::subscribe "/currentor_socket/sensor_array/position"
		std_msgs::float32multiarray
		#'position-callback
		1)
;; (ros::subscribe "/currentor_socket/sensor_array/imu"
;; 		std_msgs::float32multiarray
;; 		#'imu-callback
;; 		1)
;; (ros::subscribe "/odom"
;; 		nav_msgs::odometry
;; 		#'imu-callback
;; 		1)
(ros::subscribe "/kduino/converted_imu" ;; "/imu_data"
                sensor_msgs::imu
		#'imu-callback
                1)

(defun gained-torque
  (tor)
  (let* ((val (abs tor)))
    (* tor 8)))

(defun sensor-feedback-loop
  (&key (hz 50))
  (ros::rate hz)
  (let ((time-step (/ 1.0 hz))
	(cnt 0)
	(pub-cnt 0)
	(time 0)
	)
    (do-until-key
     (setq
      time
      (+
       time
       (setq
	time-step
	(bench2
	 (if (not (ros::ok)) (return-from nil nil))
	 (ros::sleep)
	 (ros::spin-once)
	 (send *viewer* :viewsurface :flush)
	 (x::window-main-one)
	 (cond
	  (*position-updated*
	   (send *robot* :calc-torque
		 :target-coords (list (send *robot* :rarm :end-coords)
				      (send *robot* :larm :end-coords))
		 :force-list (list #F(0 0 0) #F(0 0 0))
		 :moment-list (list #F(0 0 0) #F(0 0 0))
		 )))
	 (torque-command
	  (map float-vector
	       #'gained-torque
	       (map float-vector
		    #'*
		    (scale -1 #F(0 1 1 1 1 1 1 1
				   1 1 1 1 1 1 1))
		    (send-all *joint-list* :joint-torque)
		    *mirror-bin*
		    )))
	 (format t "[~A]~%" (if *position-updated* "receive" "skip"))
	 (format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	 (format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	 (incf cnt)
	 (if *position-updated* (incf pub-cnt))
	 (setq *position-updated* nil)
	 )))))))

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



#|
(defun sensor-feedback-loop
  (&key (hz 30))
  (ros::rate hz)
  (let ((time-step (/ 1.0 hz))
	(cnt 0)
	(pub-cnt 0)
	(time 0)
	)
    (do-until-key
     ;;(let* ((timer (instance mtimer :init)))
     (setq
      time
      (+
       time
       (bench2
	(if (not (ros::ok)) (return-from nil nil))
	(send *robot* :calc-torque
	      :target-coords (list (send *robot* :rarm :end-coords)
				   (send *robot* :larm :end-coords))
	      :force-list (list #F(0 0 0) #F(0 0 0))
	      :moment-list (list #F(0 0 0) #F(0 0 0))
	      )
	(call-function :func "setTorques2"
		       :param
		       (format nil "[~A]"
			       (float-vector2string
				(map float-vector
				     #'*
				     (scale -7 #F(0 1 1 1 0 0))
				     (send-all *joint-list* :joint-torque)
				     *mirror-bin*
				     )))
		       :spin? nil)
	;; (call-function :func "getValues" :spin? nil)
	(ros::spin-once)
	(ros::sleep)
	(map cons
	     #'(lambda (pos mir J)
		 (send j :joint-angle (* mir (rad2deg pos))))
	     (nth 0 *response-float-vector*)
	     *mirror-bin*
	     *joint-list*)
	(send *viewer* :draw-objects)
	(x::window-main-one)
	(format t "[~A]~%" (if *response-exist* "receive" "skip"))
	(format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	(format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	(incf cnt)
	(if *response-exist* (incf pub-cnt))
	(setq *response-exist* nil)
	))))))
