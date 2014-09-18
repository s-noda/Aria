#!/usr/bin/env roseus

;; (require "sample_json_publisher.l")
(require "package://aria_model/aria.l")

(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "sensor_msgs")

;;(ros::unsubscribe "/ros2http/socket_listener/reponse")

(defclass aria-interface
  :super object
  :slots (robot imu-coords joint-cnt
		joint-vector joint-list mirror-bin
		position-updated)
  )
(defmethod aria-interface
  (:init
   (&key
    ((:robot rb) (aria))
    ((:imu-coords ic)
     (let* ((bl (find-if #'(lambda (a) (string-equal (send a :name) "body_link"))
			 (send rb :links))))
       (make-cascoords
	:name "imu_coords"
	:coords
	(send
	 (send
	  (send bl :copy-worldcoords)
	  :rotate (deg2rad -90) :x)
	 :rotate (deg2rad (* 2 90)) :z)
	:parent bl)))
    ((:joint-cnt jc) 30)
    ((:joint-vector jv) (instantiate float-vector jc))
    ((:joint-list jl)
     (mapcar
      #'(lambda (name)
	  (or
	   (find-if #'(lambda (j) (string-equal name (send j :name)))
		    (send rb :joint-list))
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
    ((:mirror-bin mb)
     (list
      0
      1 1 1 -1 -1 -1 -1
      1 1 1 -1 +1 -1 +1
      1 1 1
      1 0.5 -0.5
      0 0))
    (viewer? nil)
    )
   (setq robot rb imu-coords ic joint-cnt jc
	 joint-vector jv joint-list jl mirror-bin mb)
   (send-all (send robot :joint-list) :min-angle -10000)
   (send-all (send robot :joint-list) :max-angle 10000)
   (send robot :put :imu-coords imu-coords)
   ;;
   (cond
    (viewer?
     (objects robot)
     (send *irtviewer* :change-background #F(0.9 0.9 0.9))
     (send *irtviewer* :draw-objects)))
   ;;
   (ros::roseus "aria_interface")
   ;;
   (ros::subscribe "/currentor_socket/sensor_array/position"
		   std_msgs::float32multiarray
		   #'send self :position-callback
		   1)
   (ros::subscribe "/currentor_socket/sensor_array/torque"
		   std_msgs::float32multiarray
		   #'send self :torque-callback
		   1)
   (ros::subscribe "/currentor_socket/sensor_array/mode"
		   std_msgs::float32multiarray
		   #'send self :mode-callback
		   1)
   (ros::subscribe "/kduino/converted_imu" ;; "/imu_data"
		   sensor_msgs::imu
		   #'send self :imu-callback
		   1)
   ;;
   (ros::advertise "/currentor_socket/request/mode_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/position_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/torque_vector"
		   std_msgs::float32multiarray)
   )
  (:state
   (&rest args)
   (let ((spin? (cadr (member :spin? args))))
     (cond
      (spin?
       (ros::sleep)
       (ros::spin-once)))
     (case (car args)
	   ((:potentio-vector :angle-vector)
	    (map float-vector
		 #'(lambda (j) (or (send j :get 'position) 0))
		 (send robot :joint-list)))
	   (:torque-vector
	    (map float-vector
		 #'(lambda (j) (or (send j :get 'torque) 0))
		 (send robot :joint-list)))
	   (:mode-vector
	    (map float-vector
		 #'(lambda (j) (or (send j :get 'mode) 0))
		 (send robot :joint-list)))
	   (:worldcoords
	    (copy-object (send robot :worldcoords)))
	   )))
  (:callback-org
   (msg
    &key
    (vector (send msg :data))
    (command 'position)
    (gain (map float-vector
	       #'(lambda (bin)
		   (cond
		    ((eq command 'torque) (* bin 0.1))
		    ((eq command 'position) (* bin (rad2deg 1)))
		    (t 1)))
	       mirror-bin))
    (etc-command (cond
		  ((eq command 'position) :joint-angle)
		  ((eq command 'torque) :joint-torque)))
    )
   (map cons
	#'(lambda (g j pos)
	    (send j :put command (* g pos))
	    (if etc-command
		(send j etc-command (send j :get command)))
	    )
	gain
	joint-list
	vector)
   )
  (:position-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'position)
   (setq position-updated t)
   (cond
    ((and draw? (boundp '*viewer*) *viewer*)
     (send *viewer* :draw-objects)
     (x::window-main-one)))
   )
  (:torque-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'torque)
   )
  (:mode-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'mode)
   )
  (:imu-callback
   (msg
    &key
    (draw? t) (flush nil))
   (let* ((ori (send msg :orientation))
	  (mat (quaternion2matrix
		(float-vector (send ori :w) (send ori :x)
			      (send ori :y) (send ori :z))))
	  (tmp-coords (make-coords :rot mat))
	  (move-coords (copy-object (send (send robot :get :imu-coords) :worldcoords)))
	  )
     (setq move-coords (send move-coords :transformation robot))
     (send tmp-coords :transform move-coords :local)
     (dotimes (i 3)
       (send robot :newcoords (copy-object tmp-coords))
       (send robot :worldcoords)
       )
     (cond
      ((and draw? (boundp '*viewer*) *viewer*)
       (send *viewer* :draw-objects :flush flush)
       (send (make-coords :rot mat) :draw-on :flush flush
	     :color #f(1 0 0) :size 100 :width 100)
       ))
     ))
  (:send-vector
   (&optional
    (vector (send robot :angle-vector))
    (command 'torque)
    (gain (map float-vector
	       #'(lambda (bin)
		   (cond
		    ((eq command 'torque) (* bin -9.8))
		    ((eq command 'position) (* bin (deg2rad 1)))
		    (t 1)))
	       mirror-bin))
    )
   (map cons
	#'(lambda (j v) (send j :put command v))
	(send robot :joint-list)
	vector)
   (dotimes (i (length joint-list))
     (setf (aref joint-vector i)
	   (* (elt gain i)
	      (or (send (elt joint-list i) :get command) 0))))
   (ros::publish (format nil "/currentor_socket/request/~A_vector" command)
		 (instance std_msgs::float32multiarray
			   :init
			   :data joint-vector)))
  (:angle-vector
   (&optional (vector (send robot :angle-vector)))
   (send self :send-vector vector 'position))
  (:torque-vector
   (&optional (vector (send robot :torque-vector)))
   (send self :send-vector vector 'torque))
  (:mode-vector
   (&optional (vector (scale 0 (send robot :angle-vector))))
   (send self :send-vector vector 'mode))
  )

(defun aria-init
  nil
  (setq *ri* (instance aria-interface :init))
  (setq *aria* (aria))
  (setq *robot* *aria*)
  (objects *robot*)
  (send *irtviewer* :change-background #F(0.9 0.9 0.9))
  (send *irtviewer* :draw-objects))

(defun cancel-gravity
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
	 (send *robot* :angle-vector
	       (send *ri* :state :potentio-vector :spin? nil))
	 (send *robot* :newcoords
	       (send *ri* :state :worldcoords  :spin? nil))
	 ;;(send *viewer* :viewsurface :flush)
	 (send *viewer* :draw-objects)
	 (x::window-main-one)
	 (if (send *ri* :get-val 'position-updated)
	     (send *ri* :torque-vector
		   (send *robot* :calc-torque
			 :target-coords (list (send *robot* :rarm :end-coords)
					      (send *robot* :larm :end-coords))
			 :force-list (list #F(0 0 0) #F(0 0 0))
			 :moment-list (list #F(0 0 0) #F(0 0 0))
			 )))
	 (format t "[~A]~%" (if (send *ri* :get-val 'position-updated) "receive" "skip"))
	 (format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	 (format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	 (incf cnt)
	 (if (send *ri* :get-val 'position-updated) (incf pub-cnt))
	 (send *ri* :set-val 'position-updated nil)
	 )))))))

