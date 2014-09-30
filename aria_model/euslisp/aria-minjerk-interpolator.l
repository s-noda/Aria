#!/usr/bin/env roseus

(require "package://aria_model/euslisp/aria-interface.l")
(require "package://aria_model/euslisp/bspline.l")
(require "package://eus_qp/euslisp/eiquadprog.l")

(aria-init)
(send-all (send *robot* :joint-list) :min-angle -180)
(send-all (send *robot* :joint-list) :max-angle +180)

(defvar *jlist* (append (send *robot* :rarm :joint-list)
			(send *robot* :larm :joint-list)))

(defun minjerk-bspline-interpole
  (&key
   (robot
    (cond
     ((and (boundp '*robot*) *robot*) *robot*)
     (t ;;
      (require "package://aria_model/aria.l")
      (setq *robot* (aria))
      ;;(require "package://euslisp/jskeus/irteus/demo/sample-robot-model.l")
      ;;(setq *robot* (instance sample-robot :init))
      (objects (list *robot*))
      *robot*)))
   (init (send robot :reset-pose))
   (jlist *jlist*)
   (start-av (send-all jlist :joint-angle))
   (end-av (mapcar #'(lambda (j)
		       (+ (send j :min-angle)
			  (random (- (send j :max-angle) (send j :min-angle)))))
		   jlist))
   (id-max 8)
   (recursive-order 4)
   (x-min 0.0)
   (x-max 1.0)
   (bspline
    (mapcar #'(lambda (k)
		(instance basic-spline :init
			  :id-max id-max :recursive-order recursive-order
			  :x-min x-min :x-max x-max))
	    jlist))
   (initial-state
    (instantiate float-vector (* id-max (length bspline))))
   (state-min-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :min-angle)))
		  jlist))))
   (state-max-vector
    (apply #'concatenate
	   (cons float-vector
		 (mapcar
		  #'(lambda (j) (make-list id-max :initial-element (send j :max-angle)))
		  jlist))))
   (equality-matrix-for-start/end-pos
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (send bs :calc-gain-vector-coeff-matrix-from-via-x-list (list 0.0 0.99)))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-pos
    (concatenate float-vector
		 (flatten (map cons #'list start-av end-av))))
   ;;
   (equality-matrix-for-start/end-vel
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	       (mapcar
		#'(lambda (x) (send bs :calc-delta-coeff-vector x :n 1))
		(list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-vel
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (equality-matrix-for-start/end-acc
    (matrix-append
     (map cons
	  #'(lambda (bs st ed)
	      (make-matrix
	       2 id-max
	      (mapcar
	       #'(lambda (x) (send bs :calc-delta-coeff-vector x :n 2))
	       (list 0.0 0.99))))
	  bspline start-av end-av)
     '(1 1)))
   (equality-coeff-for-start/end-acc
    (scale 0 (concatenate float-vector
			  (flatten (map cons #'list start-av end-av)))))
   ;;
   (eval-weight-matrix
    (let* ((mat
	    (matrix-append
	     (mapcar
	      #'(lambda (rate)
		  (matrix-append
		   (mapcar
		    #'(lambda (bs)
			(make-matrix
			 1 id-max
			 (list
			  (scale
			   1e-3
			   (send bs :calc-delta-coeff-vector
				 (+ x-min (* rate (- x-max x-min)))
				 :n 3)))))
		    bspline)
		   '(1 1)))
	      '(0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0))
	     '(1 0))))
      (m* (transpose mat) mat)))
   (cnt 30)
   (x-step (/ (- x-max x-min) (* 1.0 cnt)))
   (x-buf x-min)
   (ret (solve-eiquadprog
	 :debug? t
	 :initial-state initial-state
	 :eval-weight-matrix eval-weight-matrix
	 :state-min-vector state-min-vector
	 :state-max-vector state-max-vector
	 :equality-vector
	 (concatenate float-vector
		      equality-coeff-for-start/end-pos
		      equality-coeff-for-start/end-vel
		      equality-coeff-for-start/end-acc)
	 :equality-matrix
	 (matrix-append
	  (list equality-matrix-for-start/end-pos
		equality-matrix-for-start/end-vel
		equality-matrix-for-start/end-acc)
	  '(1 0))
	 ))
   )
  (if (null ret) (setq ret initial-state))
  (format t "   --- ~A x ~A = ~A variables~%" id-max (length start-av) (length initial-state))
  (let* ((retl (list (cons :gain ret))) p dp ddp (id) tau)
    (setq x-buf x-min)
    (while (<= x-buf x-max)
      (setq id 0)
      (mapcar
       #'(lambda (bs js)
	   (list
	    (send js :joint-angle
		  (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :p (send bs :calc x-buf (subseq ret id (+ id id-max))))
	    (send js :put :dp (send bs :calc-delta x-buf (subseq ret id (+ id id-max)) :n 1))
	    (send js :put :ddp (send bs :calc-delta x-buf (subseq ret id (setq id (+ id id-max))) :n 2))
	    ))
       bspline jlist)
      (push (send-all jlist :get :ddp) ddp)
      (push (send-all jlist :get :dp) dp)
      (push (send-all jlist :get :p) p)
      (send *robot* :calc-torque-from-vel-acc
	    :jvv (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :dp) 0)))
		      (cdr (send robot :links)))
	    :jav (map float-vector
		      #'(lambda (j) (deg2rad (or (send j :get :ddp) 0)))
		      (cdr (send robot :links))))
      (push (send-all jlist :joint-torque) tau)
      (setq x-buf (+ x-buf x-step))
      (send *viewer* :draw-objects)
      (x::window-main-one)
      (unix:usleep (round (* 0.01 1000 1000))))
    (push (cons :p (reverse p)) retl)
    (push (cons :dp (reverse dp)) retl)
    (push (cons :ddp (reverse ddp)) retl)
    (push (cons :tau (reverse tau)) retl)
    (format t "  [dif] |~A| = ~A~%"
	    (map float-vector #'- end-av (send-all jlist :joint-angle))
	    (norm (map float-vector #'- end-av (send-all jlist :joint-angle))))
    retl
    )
  )

(defun interpole-string-callback
  (msg)
  (let* ((args (read-from-string (send msg :data)))
	 (av (car args))
	 (tm (cadr args)))
    (map cons
	 #'(lambda (j av) (send j :put :p av))
	 (send *robot* :joint-list) av)
    (format t "[start-interpolation] ~A~%" (send msg :data))
    (interpolator-callback (send-all *jlist* :get :p) tm)))

(defun interpole-jointstate-callback
  (msg)
  (let* ((j))
    (mapcar #'(lambda (j) (send j :put :p (send j :joint-angle)))
	    (send *robot* :joint-list))
    (map cons
	 #'(lambda (n p)
	     (setq j (find-if #'(lambda (j) (string-equal n (send j :name)))
			      (send *robot* :joint-list)))
	     (if j (send j :put :p (rad2deg p))))
	 (send msg :name)
	 (send msg :position))
    (interpolator-callback
     (map cons
	  #'(lambda (j) (or (send j :get :p) (send j :joint-angle)))
	  *jlist*)
     5000)))

(defun interpolator-callback
  (av tm)
  (let* ((min-tm 50.0)
	 (cnt (/ tm min-tm))
	 (traj
	  (minjerk-bspline-interpole
	   :init nil
	   :jlist *jlist*
	   :start-av (send-all *jlist* :joint-angle)
	   :end-av av
	   :cnt cnt
	   ;; :x-max (* 1e-3 tm)
	   ))
	 rate mtm)
    (setq mtm (instance mtimer :init))
    (dotimes (i (+ cnt 1))
      (print 'pub)
      (send *ri* :_angle-vector (nth i (cdr (assoc :p traj))))
      (unix:usleep (round (* 0.5 min-tm 1000)))
      (send *ri* :torque-vector (nth i (cdr (assoc :tau traj))))
      (unix:usleep (round (* 0.5 min-tm 1000))))
    (format t "   ~A~%" (send mtm :stop))
    ))

(ros::subscribe "/aria_minjerk_interpolator/request/string"
		std_msgs::string
		#'interpole-string-callback
		10)

(ros::subscribe "/aria_minjerk_interpolator/request/joint_state"
		sensor_msgs::jointstate
		#'interpole-jointstate-callback
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