
(require "package://aria_model/aria.l")
;; (require "package://eus_eigen/euslisp/eus_eigen.l")

(defvar *robot* (aria))
(objects (list *robot*))

(defun calc-ddq-coeff-matrix ;; calc H of Hddq + b = t + Jf
  (&key
   (robot *robot*)
   (root-link (car (send robot :links)))
   (root-pos (send (send root-link :worldcoords) :transform-vector
		   (send root-link :get-val 'acentroid)))
   (all-links (remove root-link (send robot :links)))
   (q (send robot :angle-vector))
    ;;(send-all (send-all all-links :joint) :joint-angle))
   (dq (scale 0 q))
   (root-angular-velocity (float-vector 0 0 0))
   (root-spacial-velocity (float-vector 0 0 0))
   (base-torque? nil)
   ;;  buf
   (ddq (scale 0 dq))
   (root-angular-acceleration (scale 0 root-angular-velocity))
   (root-spacial-acceleration (scale 0 root-spacial-velocity))
   (ddq-coeff
    (make-matrix (+ (length all-links) (if base-torque? 6 0))
		 (+ (length all-links) (if base-torque? 6 0))))
   (b (instantiate float-vector (+ (length all-links) (if base-torque? 6 0))))
   (bbuf (copy-seq b))
   (x 0) (y 0)
   (calc-torque-buffer-args (send robot :calc-torque-buffer-args))
   )
  (labels
   ((set-ddq-coeff-column
     (&key (set-b? nil) (id 0) f)
     (send robot
	   :calc-torque-from-vel-acc
	   :root-angular-velocity root-angular-velocity
	   :root-spacial-velocity root-spacial-velocity
	   :root-spacial-acceleration root-spacial-acceleration
	   :root-angular-acceleration root-angular-acceleration
	   :jvv (send-all (cdr (send robot :links)) :get :dq)
	   :jav (send-all (cdr (send robot :links)) :get :ddq)
	   :calc-torque-buffer-args calc-torque-buffer-args)
     (setq id 0)
     (dolist (jt (send-all (send-all all-links :joint) :joint-torque))
       (setf (aref bbuf id) jt) (incf id))
     (cond
      (base-torque?
       (setq f (send root-link :force))
       (dotimes (i 3) (setf (aref bbuf id) (aref f i)) (incf id))
       (setq f (v- (send root-link :moment)
		   (scale
		    1e-3
		    (v* root-pos
			(send root-link :force)))))
       (dotimes (i 3) (setf (aref bbuf id) (aref f i)) (incf id))))
     (cond
      (set-b? (dotimes (i (length bbuf)) (setf (aref b i) (aref bbuf i))))
      (t
       (setq y 0)
       (setq f (v- bbuf b))
       (dotimes (i (length f)) (setf (aref ddq-coeff y x) (aref f i)) (incf y))
       (incf x)))))
   ;;(send robot :angle-vector (copy-seq q))
   (map cons #'(lambda (l q dq ddq)
		 (send l :put :q q)
		 (send l :put :dq dq)
		 (send l :put :ddq ddq)
		 (send (send l :joint) :joint-angle q))
	all-links q dq ddq)
   (set-ddq-coeff-column :set-b? t)
   ;;
   (cond
    (base-torque?
     (dotimes (i 3)
       (setf (aref root-spacial-acceleration i) 1)
       (set-ddq-coeff-column)
       (setf (aref root-spacial-acceleration i) 0))
     (dotimes (i 3)
       (setf (aref root-angular-acceleration i) 1)
       (set-ddq-coeff-column)
       (setf (aref root-angular-acceleration i) 0))
     ))
   ;;
   (dolist (l all-links)
     (send l :put :ddq 1)
     (set-ddq-coeff-column)
     (send l :put :ddq 0))
   ;;
   ddq-coeff
   ))

(defun debug-calc-ddq-coeff-matrix-transpose
  (&rest
   args
   &key
   (robot *robot*)
   (q (map float-vector
	   #'(lambda (v) (- (* 2 (random 90.0)) 90))
	   (send robot :angle-vector)))
   (dq (map float-vector
	    #'(lambda (v) (- (* 2 (random 1.0)) 1))
	    (send robot :angle-vector)))
   (root-angular-velocity (scale 0.1 (random-vector 1.0)))
   (root-spacial-velocity (scale 0.1 (random-vector 1.0)))
   (mat (bench (apply #'calc-ddq-coeff-matrix
		      (append
		       (list :robot robot :q q :dq dq
			     :root-spacial-velocity root-spacial-velocity
			     :root-angular-velocity root-angular-velocity)
		       args)))))
  (norm
   (send
    (format-array (m+ mat (transpose mat)))
    :get-val 'entity))
  )

(defun debug-calc-ddq-coeff-matrix-torque-check
  (&rest
   args
   &key
   (robot *robot*)
   (all-links (cdr (send robot :links)))
   (q (map float-vector
	   #'(lambda (v) (- (* 2 (random 90.0)) 90))
	   all-links))
   (dq (map float-vector
	    #'(lambda (v) (- (* 2 (random 1.0)) 1))
	    all-links))
   (ddq (map float-vector
	     #'(lambda (v) (- (* 2 (random 1.0)) 1))
	     all-links))
   (root-angular-velocity (scale 0.1 (random-vector 1.0)))
   (root-spacial-velocity (scale 0.1 (random-vector 1.0)))
   (root-angular-acceleration (scale 0.1 (random-vector 1.0)))
   (root-spacial-acceleration (scale 0.1 (random-vector 1.0)))
   (base-torque? nil)
   (ddq-coeff
    (make-matrix (+ (length all-links) (if base-torque? 6 0))
		 (+ (length all-links) (if base-torque? 6 0))))
   (b (instantiate float-vector (+ (length all-links) (if base-torque? 6 0))))
   (mat (apply #'calc-ddq-coeff-matrix
	       (append
		(list :robot robot :q q :dq dq
		      :all-links all-links
		      :root-spacial-velocity root-spacial-velocity
		      :root-angular-velocity root-angular-velocity
		      :ddq-coeff ddq-coeff
		      :b b
		      :base-torque? base-torque?
		      )
		nil)))
   (ans (send robot :calc-torque-from-vel-acc
	      :root-angular-velocity root-angular-velocity
	      :root-spacial-velocity root-spacial-velocity
	      :root-spacial-acceleration root-spacial-acceleration
	      :root-angular-acceleration root-angular-acceleration
	      :jvv dq
	      :jav ddq))
   (pre (v+ (transform ddq-coeff ddq) b))
   (debug? t)
   )
  (if base-torque? (setq pre (subseq pre 6)))
  (format debug? "[torque-check]~% answer = ~A~% predict = ~A~% diff = ~A~%"
	  ans pre (v- ans pre))
  (/  (norm (v- ans pre)) (norm ans))
  )

(send-all (send *robot* :joint-list) :min-angle *-inf*)
(send-all (send *robot* :joint-list) :max-angle *inf*)
(mapcar
 #'(lambda (l)
     (if (< (send l :weight) 1) (send l :weight 1))
     (if (< (norm (send (send l :inertia-tensor) :get-val 'entity)) 100)
	 (send l :inertia-tensor (scale-matrix 1e+5 (unit-matrix 3))))
     (if (< (norm (send l :get-val 'acentroid)) 1e-3)
	 (send l :set-val 'acentroid (float-vector 1e-3 1e-3 1e-3))))
 (send *robot* :links))

(defun debug-calc-ddq-coeff-matrix-simulation
  (&rest
   args
   &key
   (robot *robot*)
   (all-links ;;(cdr (send robot :links)))
    (cdr (send robot :rarm :links)))
   (q (map float-vector
	   #'(lambda (v) (* 1 (- (* 2 (random 180.0)) 180)))
	   all-links))
   (dq (map float-vector
	    #'(lambda (v) (- (* 2 (random 1.0)) 1))
	    all-links))
   (ddq (map float-vector
	     #'(lambda (v) (- (* 2 (random 1.0)) 1))
	     all-links))
   (root-angular-velocity (scale 0 (random-vector 1.0)))
   (root-spacial-velocity (scale 0 (random-vector 1.0)))
   (root-angular-acceleration (scale 0 (random-vector 1.0)))
   (root-spacial-acceleration (scale 0 (random-vector 1.0)))
   (sim-time-step 0.01)
   (sim-time-max 1000)
   (sim-time-buf 0)
   (tau (instantiate float-vector (length all-links)))
   ;;
   (friction-matrix
    (scale-matrix 0.1 (unit-matrix (length all-links))))
   (inertia-matrix
    (scale-matrix 0.001 (unit-matrix (length all-links))))
   (base-torque? nil)
   (ddq-coeff
    (make-matrix (+ (length all-links) (if base-torque? 6 0))
		 (+ (length all-links) (if base-torque? 6 0))))
   (b (instantiate float-vector (+ (length all-links) (if base-torque? 6 0))))
   ddq#
   (max-vel 1.5)
   (min-vel -1.5)
   ;;
   (animate-time-step (max 0.2 sim-time-step))
   (animate-cnt (round (/ animate-time-step sim-time-step)))
   (animate-timer (instance mtimer :init))
   ;;
   (ret q)
   )
  (do-until-key
   (if(> (setq sim-time-buf (+ sim-time-buf sim-time-step))
	 sim-time-max) (return-from nil nil))
    (calc-ddq-coeff-matrix
     :robot robot :q q :dq dq :all-links all-links
     :root-spacial-velocity root-spacial-velocity
     :root-angular-velocity root-angular-velocity
     :ddq-coeff ddq-coeff :b b :base-torque? base-torque?)
    ;; (print (calc-eigen-determinant ddq-coeff))
    (setq ddq# (inverse-matrix ddq-coeff))
    ;; (calc-eigen-inverse ddq-coeff))
    (if ddq#
	(setq ddq
	      (transform
	       ddq#
	       (v- (v- (v- tau b)
		       (transform friction-matrix dq))
		   (transform inertia-matrix ddq)))))
    (setq dq (map float-vector
		  #'(lambda (v) (min max-vel (max min-vel v)))
		  (v+ dq (scale sim-time-step ddq))))
    (setq q (v+ q (scale (rad2deg sim-time-step) dq)))
    ;; (format t "  (~A)  ddq=~A~%" sim-time-buf ddq)
    (cond
     ((minusp (decf animate-cnt))
      (setq animate-cnt
	    (round (/ animate-time-step sim-time-step)))
      (format t " time = ~A x~A~%"
	      sim-time-buf
	      (/ animate-time-step (send animate-timer :stop)))
      (send animate-timer :start)
      (x::window-main-one)
      (send *viewer* :draw-objects)
      ))
    )
  ret
  )

