#!/usr/bin/env roseus

;; (require "sample_json_publisher.l")
(require "package://aria_model/aria.l")

(ros::load-ros-manifest "nav_msgs")
(ros::load-ros-manifest "sensor_msgs")

;;(ros::unsubscribe "/ros2http/socket_listener/reponse")

(defvar *ri*)
(defvar *aria*)
(defvar *robot*)

;; added by ssb
(defvar *root-accel* (float-vector 0 0 0))
;; added by ssb

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
    ((:imu-coords ic) (gen-aria-imu-coords rb))
    ((:joint-cnt jc) 30)
    ((:joint-vector jv) (instantiate float-vector jc))
    ((:joint-list jl)
     (mapcar
      #'(lambda (name)
	  (or
	   (find-if #'(lambda (j) (string-equal name (send j :name)))
		    (send rb :joint-list))
	   (instance rotational-joint :init
		     :child-link (make-coords)
		     :min -1e+6
		     :max +1e+6)))
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
      1 1))
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
   ;; test ssb
   (ros::subscribe "/currentor_socket/sensor_array/position"
		   ;;   (ros::subscribe "/currentor_socket/sensor_array/debug"
		   ;; test ssb
		   std_msgs::float32multiarray
		   #'send self :position-callback
		   1)
   (ros::subscribe "/currentor_socket/sensor_array/velocity"
		   std_msgs::float32multiarray
		   #'send self :velocity-callback
		   1)
   (ros::subscribe "/currentor_socket/sensor_array/torque"
		   std_msgs::float32multiarray
		   #'send self :torque-callback
		   1)
   (ros::subscribe "/currentor_socket/sensor_array/temperature"
		   std_msgs::float32multiarray
		   #'send self :temperature-callback
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
   ;; added by ssb
   (ros::subscribe "/root_force/virtual_array/spacial" std_msgs::Float32MultiArray #'send self :root-accel-callback)
   ;; added by ssb
   (ros::advertise "/currentor_socket/request/mode_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/position_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/torque_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/wheel_torque_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/torque_max_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/torque_min_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/currentor_socket/request/torque_combined_vector"
		   std_msgs::float32multiarray)
   (ros::advertise "/aria_interpolator/position"
		   std_msgs::string)
   (ros::advertise "/aria_minjerk_interpolator/request/string"
		   std_msgs::string)
   )
  (:state
   (&rest args)
   (let ((spin? (if (member :spin? args)
		    (cadr (member :spin? args)) t)))
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
		    ((eq command 'torque) (/ bin -9.8))
		    ((eq command 'position) (* bin (rad2deg 1)))
		    ((eq command 'velocity) (* bin (rad2deg 1)))
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
  (:velocity-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'velocity)
   )
  (:torque-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'torque)
   )
  (:temperature-callback
   (msg &key (draw? nil))
   (send self :callback-org msg :command 'temperature)
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
	  ;; (rpy (map cons #'* '(1 -1 1) (car (rpy-angle mat)))) ;; agry fix
	  ;; (tmp-coords (make-coords :rpy rpy))
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
  ;; added by ssb
  (:root-accel-callback
   (msg)
   (setq *root-accel* (send msg :data)))
  ;; added by ssb
  (:send-vector
   (&optional
    (vector (send robot :angle-vector))
    (command 'torque)
    (gain (map float-vector
	       #'(lambda (bin)
		   (let* ((bin-1 (if (< (abs bin) 1e-6) 0 (/ 1.0 bin))))
		     (cond
		      ((eq command 'torque) (* bin-1 -9.8))
		      ((eq command 'torque_max) (* bin-1 -9.8))
		      ((eq command 'torque_min) (* bin-1 -9.8))
		      ((eq command 'torque_combined) (* bin-1 -9.8))
		      ((eq command 'position) (* bin-1 (deg2rad 1)))
		      (t 1))))
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
  (:mirror-bin
   nil
   (map cons #'(lambda (j b) (send j :put :bin b)) joint-list mirror-bin)
   (mapcar #'(lambda (j) (or (send j :get :bin) 0)) (send robot :joint-list)))
  (:_angle-vector
   (&optional (vector (send robot :angle-vector)) (tm 5000) ret)
   (setf (aref joint-vector (- (length joint-vector) 1))
	 (* 1e-3 tm))
   (setq ret (send self :send-vector vector 'position))
   (setf (aref joint-vector (- (length joint-vector) 1))
	 0)
   ret
   )
  (:angle-vector
   (&optional (vector (send robot :angle-vector)) (tm 5000))
   (ros::publish "/aria_minjerk_interpolator/request/string";;"/aria_interpolator/position"
		 (instance std_msgs::string
			   :init
			   :data (format nil "(~A ~A)" vector tm))))
  ;; (:wheel-torque-vector
  ;;  (&optional (vector (float-vector 0 0))
  ;; 	      (torque-vector (scale 0 (send robot :angle-vector))))
  ;;  (map cons
  ;; 	#'(lambda (j v) (send j :put 'torque v))
  ;; 	(subseq joint-list (- (length joint-list) 2))
  ;; 	vector)
  ;;  (send self :torque-vector torque-vector))
  (:wheel-torque-vector
   (&optional (vector (float-vector 0 0)))
   (ros::publish "/currentor_socket/request/wheel_torque_vector"
		 (instance std_msgs::float32multiarray
			   :init
			   :data vector)))
  ;; (ros::publish "/aria_minjerk_interpolator/request/string";;"/aria_interpolator/position"
  ;; 		 (instance std_msgs::string
  ;; 			   :init
  ;; 			   :data (format nil "(~A ~A)" vector tm))))
  (:torque-vector
   (&optional (vector (send robot :torque-vector)))
   (send self :send-vector vector 'torque))
  ;; (:torque-max-vector
  ;;  (&optional (vector (send robot :torque-vector)))
  ;;  (send self :send-vector vector 'torque_max))
  ;; (:torque-min-vector
  ;;  (&optional (vector (send robot :torque-vector)))
  ;;  (send self :send-vector vector 'torque_min))
  (:torque-combined-vector
   (&optional (vector (send robot :torque-vector)))
   (send self :send-vector vector 'torque_combined))
  (:mode-vector
   (&optional (vector (scale 0 (send robot :angle-vector))))
   (send self :send-vector vector 'mode))
  ;;)
  )

(defun gen-aria-imu-coords
  (robot)
  (let* ((bl (find-if #'(lambda (a) (string-equal (send a :name) "body_link"))
		      (send robot :links)))
	 (pl (find-if #'(lambda (a) (string-equal (send a :name) "hip2_link"))
		      (send robot :links))))
    ;; (make-cascoords
    ;; 	:name "imu_coords"
    ;; 	:coords
    ;; 	(send
    ;; 	 (send
    ;; 	  (send bl :copy-worldcoords)
    ;; 	  :rotate (deg2rad -90) :x)
    ;; 	 :rotate (deg2rad (* 2 90)) :z)
    ;; 	:parent bl)
    (make-cascoords
     :name "imu_coords"
     :coords
     (send
      (send
       (send
	(send bl :copy-worldcoords)
	:translate #F(0 0 -550) :world)
       :rotate (deg2rad -90) :x)
      :rotate (deg2rad (* -2 90)) :z)
     :parent pl)
    ))

(defun aria-init
  (&key
   (viewer? t))
  (setq *ri* (or *ri* (instance aria-interface :init)))
  (setq *aria* (or *aria* *robot* (aria)))
  (setq *robot* (or *robot* *aria*))
  (send-all (send *robot* :joint-list) :min-angle -10000)
  (send-all (send *robot* :joint-list) :max-angle 10000)
  (cond
   (viewer?
    (objects *robot*)
    (send *irtviewer* :change-background #F(0.9 0.9 0.9))
    (send *irtviewer* :draw-objects))))

(defun model2real
  (&key (sleep-time 1000))
  (send *ri* :_angle-vector
	(send *robot* :angle-vector) sleep-time)) ;; 3000))

(defun real2model
  (&key (draw? t))
  (send *robot* :angle-vector
	(send *ri* :state :potentio-vector))
  (send *robot* :newcoords
	(send *ri* :state :worldcoords :spin? nil))
  (if draw? (send *viewer* :draw-objects))
  )

(defun tablet-pose
  nil
  (send *robot* :angle-vector #f(0.0 1.69744 32.5919 -95.2269 -64.5411 15.3429 -28.6477 -37.4261 -24.0 -6.98202 -11.2998 -70.5728 2.45552 -11.4646 -42.0954 2.31269 0.052183 10.0 -6.02618 1.07119 2.86203 -2.36763)))


(defmethod bodyset-link
  (:force (&optional f) (if f (setq force f) force))
  (:moment (&optional m) (if m (setq moment m) moment))
  (:angular-velocity (&optional av)
		     (if av (setq angular-velocity av) angular-velocity))
  (:angular-acceleration (&optional aa)
			 (if aa (setq angular-acceleration aa) angular-acceleration))
  (:spacial-velocity (&optional sv)
		     (if sv (setq spacial-velocity sv) spacial-velocity))
  (:spacial-acceleration (&optional sa)
			 (if sa (setq spacial-acceleration sa) spacial-acceleration))
  )

(defmethod robot-model
  (:torque-vector2
   (&key (contact-states
	  (mapcar
	   #'(lambda (k)
	       (list
		(cons :link (send self k :end-coords :parent))
		(cons :worldcoords
		      (copy-object (send self k :end-coords :worldcoords)))
		(cons :ext-force (float-vector 0 0 0))
		(cons :ext-moment (float-vector 0 0 0))))
	   '(:rarm :larm)))
;	 (gravity; #F(0 0 0))
;	  (scale (* 1e-3 9.8 (send self :weight))
;		 #F(0 0 -1)))
	 (root-angular-velocity (float-vector 0 0 0))
	 (root-spacial-velocity (float-vector 0 0 0))
	 (root-angular-acceleration (float-vector 0 0 0))
	 (root-spacial-acceleration (float-vector 0 0 0))
	 (root-link (car (send self :links)))
         ret
	 (debug-view nil)
         (jvv (instantiate float-vector (calc-target-joint-dimension joint-list)))
         (jav (instantiate float-vector (calc-target-joint-dimension joint-list)))
         (calc-torque-buffer-args (send self :calc-torque-buffer-args))
	 (debug? nil))
;   (send (car (send self :links)) :force gravity)
   (mapcar #'(lambda (cs)
               (send (cdr (assoc :link cs)) :ext-force
		     (or (cdr (assoc :ext-force cs)) (float-vector 0 0 0)))
               (let* ((moment-offset
		       (v* (scale 1e-3
				  (send (cdr (assoc :worldcoords cs))
					:worldpos))
			   (or (cdr (assoc :ext-force cs)) (float-vector 0 0 0)))))
		 (send (cdr (assoc :link cs)) :ext-moment
		       (v+ moment-offset (or (cdr (assoc :ext-moment cs))
					     (float-vector 0 0 0))))))
	   contact-states)
   (setq
    ret
    (send self :calc-torque-from-vel-acc
	  :debug-view debug-view
	  :root-angular-acceleration root-angular-acceleration
	  :root-angular-velocity root-angular-velocity
	  :root-spacial-acceleration root-spacial-acceleration
	  :root-spacial-velocity root-spacial-velocity
	  :jvv jvv
	  :jav jav
	  :calc-torque-buffer-args calc-torque-buffer-args))
   (cond
    ((and debug?
	  (>
	   (norm
	    (concatenate
	     float-vector
	     (send root-link :force)
	     (send root-link :moment)))
	   1))
     (format t "[torque-vector2] root force detected ~A >1~%"
	     (concatenate
	      float-vector
	      (send root-link :force)
	      (send root-link :moment)))))
   ret
   )
  )


(defun cancel-gravity
;;  (&key (hz 50)
  (&key (hz 20)
	(send-torque? t)
;; added by ssb
;;	(root-accel (float-vector 0 0 0))
;; added by ssb
	(torque-limit?))
  (if (and torque-limit? (not (numberp torque-limit?)))
      (setq torque-limit? 0.001))
  (ros::rate hz)
  (let ((time-step (/ 1.0 hz))
	(cnt 0)
	(pub-cnt 0)
	(time 0)
	tv
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
	 (cond
	  ((send *ri* :get-val 'position-updated)
	   ;; (setq tv
	   ;; 	 (send *robot* :calc-torque
	   ;; 	       :target-coords (list (send *robot* :rarm :end-coords)
	   ;; 				    (send *robot* :larm :end-coords))
	   ;; 	       :force-list (list #F(0 0 0) #F(0 0 0))
	   ;; 	       :moment-list (list #F(0 0 0) #F(0 0 0))
	   ;; 	       ))
	   (setq tv
		 (send *robot* :torque-vector2
		       :contact-states
		       (mapcar
			#'(lambda (cs f m)
			    (list
			     (cons :link (send cs :parent))
			     (cons :worldcoords (send cs :copy-worldcoords))
			     (cons :ext-force f)
			     (cons :ext-moment m)))
			(list (send *robot* :rarm :end-coords)
			      (send *robot* :larm :end-coords))
			(list #F(0 0 0) #F(0 0 0))
			(list #F(0 0 0) #F(0 0 0)))
		       :root-angular-velocity (float-vector 0 0 0)
		       :root-spacial-velocity (float-vector 0 0 0)
		       :root-angular-acceleration (float-vector 0 0 0)
;;		       :root-spacial-acceleration (float-vector 0 0 0)
		       :root-spacial-acceleration *root-accel*
		       ))
	   (if send-torque? (send *ri* :torque-vector tv))
	   (if torque-limit?
	       (progn
		 ;;(send (nth 21 (send *ri* :get-val 'joint-list))
		 ;;      :put 'torque_combined 4.0)
		 ;;(send (nth 22 (send *ri* :get-val 'joint-list))
		 ;;      :put 'torque_combined 4.0)
		 (send *ri* :torque-combined-vector tv)))
	       ;; (and (send *ri* :torque-min-vector
	       ;; 		  (map float-vector
	       ;; 		       #'(lambda (v m)
	       ;; 			   (- v torque-limit?))
	       ;; 			   ;; (+ (if (> (* m v) 0)
	       ;; 			   ;; 	  (* -1.0 torque-limit?)
	       ;; 			   ;; 	(* +1.0 torque-limit?)) v))
	       ;; 		       tv (send *ri* :mirror-bin)))
	       ;; 	    (send *ri* :torque-max-vector
	       ;; 		  (map float-vector
	       ;; 		       #'(lambda (v m)
	       ;; 			   (+ v torque-limit?))
	       ;; 			   ;; (+ (if (> (* m v) 0)
	       ;; 			   ;; 	  (* +1.0 torque-limit?)
	       ;; 			   ;; 	(* -1.0 torque-limit?)) v))
	       ;; 		       tv (send *ri* :mirror-bin))))
	   ))
	 ;; (map cons
	 ;;      #'(lambda (v m j)
	 ;; 	  (format t " ~Ax~A E [~A ~A] at ~A~%"
	 ;; 		  v m
	 ;; 		  (* (if (> (* m v) 0)
	 ;; 			 (- 1.0 torque-limit?)
	 ;; 		       (+ 1.0 torque-limit?)) v)
	 ;; 		  (* (if (> (* m v) 0)
	 ;; 			 (+ 1.0 torque-limit?)
	 ;; 		       (- 1.0 torque-limit?)) v)
	 ;; 		  (send j :name)))
	 ;;      tv (send *ri* :mirror-bin) (send *robot* :joint-list))
	 (format t "[~A]~%" (if (send *ri* :get-val 'position-updated) "receive" "skip"))
	 (format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	 (format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	 (incf cnt)
	 (if (send *ri* :get-val 'position-updated) (incf pub-cnt))
	 (send *ri* :set-val 'position-updated nil)
	 )))))))

#|

(send *robot* :angle-vector
      #F(0.0 -2.21381 -0.17029 -120.0 -0.071416 4.33423 30.0 -0.598775 0.0 0.89541 0.499902 -120.0 0.049437 2.87301 -30.0 2.09296 -0.348825 0.085145 -1.25797 0.543833 -0.126345 -70.0))


(send-all (send *robot* :joint-list) :put :mode 3)
(send (send *robot* :rarm :shoulder-y) :put :mode 2)
;;(send (send *robot* :rarm :shoulder-r) :put :mode 2)
;;(send (send *robot* :rarm :shoulder-p) :put :mode 2)
(send (send *robot* :rarm :elbow-y) :put :mode 2)
;;(send (send *robot* :larm :shoulder-y) :put :mode 2)
;;(send (send *robot* :larm :shoulder-r) :put :mode 2)
;;(send (send *robot* :larm :shoulder-p) :put :mode 2)
(send (send *robot* :larm :elbow-y) :put :mode 2)
(send (send *robot* :head :neck-p) :put :mode 2)
(send (send *robot* :head :neck-y) :put :mode 2)
(send (send *robot* :head :neck-r) :put :mode 2)
(send (send *robot* :torso :waist-r) :put :mode 2)
(send (send *robot* :torso :waist-y) :put :mode 2)
(send (send *robot* :torso :waist-p) :put :mode 2)
(send *ri* :mode-vector (send-all (send *robot* :joint-list) :get :mode))

(send-all (send *ri* :get-val 'joint-list) :put 'mode 2)
(send *ri* :mode-vector (fill (copy-object (send *robot* :angle-vector)) 2))

(send *robot* :angle-vector #f(0.0 -2.21381 -0.17029 -120.0 -0.071416 4.33423 30.0 -0.598775 0.0 0.89541 -81.6 -110.4 -4.8 -4.8 120.0 2.09296 -0.348825 0.085145 -1.25797 0.543833 -0.126345 -70.0))
(model2real)

(send *robot* :angle-vector #f(0.0 -2.21381 -0.17029 -120.0 -0.071416 4.33423 30.0 -0.598775 0.0 0.89541 -81.6 -19.2 -4.8 -4.8 64.8 2.09296 -0.348825 0.085145 -1.25797 0.543833 -0.126345 -70.0))
(send *ri* :_angle-vector (send *robot* :angle-vector))


;; chop
(dotimes (i 10)
  (send *ri* :mode-vector
	(fill (copy-object (send *robot* :angle-vector)) 2))
  (send *ri* :torque-max-vector
	(fill (copy-object (send *robot* :angle-vector)) 100)))


(dotimes (i 10)
  (send-all (send *robot* :joint-list) :put :mode 2)
  ;;(send-all (send *robot* :rarm :joint-list) :put :mode 3)
  (send *ri* :mode-vector
	(send-all (send *robot* :joint-list) :get :mode))
  )


(unix:sleep 5)
(dotimes (i 10)
  (send *robot* :angle-vector #f(0.0 -2.21381 -0.17029 -120.0 -0.071416 4.33423 30.0 -0.598775 0.0 0.89541 -81.6 -19.2 -4.8 -4.8 64.8 2.09296 -0.348825 0.085145 -1.25797 0.543833 -0.126345 -70.0))
  (send *ri* :_angle-vector (send *robot* :angle-vector)))
