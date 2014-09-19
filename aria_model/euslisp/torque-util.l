
(require "package://aria_model/aria.l")

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
	   :jvv dq
	   :jav ddq
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
   (map cons #'(lambda (l av) (send (send l :joint) :joint-angle av)) all-links q)
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
   (dotimes (i (length ddq))
     (setf (aref ddq i) 1)
     (set-ddq-coeff-column)
     (setf (aref ddq i) 0))
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
   (mat (bench (apply #'calc-ddq-coeff-matrix
		      (append
		       (list :robot robot :q q :dq dq
			     :all-links all-links
			     :root-spacial-velocity root-spacial-velocity
			     :root-angular-velocity root-angular-velocity
			     :ddq-coeff ddq-coeff
			     :b b
			     :base-torque? base-torque?
			     )
		       args))))
   (ans (send robot :calc-torque-from-vel-acc
	      :root-angular-velocity root-angular-velocity
	      :root-spacial-velocity root-spacial-velocity
	      :root-spacial-acceleration root-spacial-acceleration
	      :root-angular-acceleration root-angular-acceleration
	      :jvv dq
	      :jav ddq))
   (pre (v+ (transform ddq-coeff ddq) b))
   )
  (if base-torque? (setq pre (subseq pre 6)))
  (format t "[torque-check]~% answer = ~A~% predict = ~A~% diff = ~A~%"
	  ans pre (v- ans pre))
  (/  (norm (v- ans pre)) (norm ans))
  )

(send-all (send *robot* :joint-list) :min-angle *-inf*)
(send-all (send *robot* :joint-list) :max-angle *inf*)
(mapcar
 #'(lambda (l)
     (if (< (send l :weight) 1) (send l :weight 1))
     (if (< (norm (send (send l :inertia-tensor) :get-val 'entity)) 100)
	 (send l :inertia-tensor (scale-matrix 1e+5 (unit-matrix 3)))))
 (send *robot* :links))

(defun debug-calc-ddq-coeff-matrix-simulation
  (&rest
   args
   &key
   (robot *robot*)
   (all-links (cdr (send robot :links)))
   (q (map float-vector
	   #'(lambda (v) (- (* 2 (random 90.0)) 90))
	   all-links))
   (dq (map float-vector
	    #'(lambda (v) 0) ;;(- (* 2 (random 1.0)) 1))
	    all-links))
   (ddq (map float-vector
	     #'(lambda (v) (- (* 2 (random 1.0)) 1))
	     all-links))
   (root-angular-velocity (scale 0 (random-vector 1.0)))
   (root-spacial-velocity (scale 0 (random-vector 1.0)))
   (root-angular-acceleration (scale 0 (random-vector 1.0)))
   (root-spacial-acceleration (scale 0 (random-vector 1.0)))
   (sim-time-step 0.01)
   (sim-time-max 10)
   (sim-time-buf 0)
   (tau (scale 0 q))
   ;;
   (friction-matrix (scale-matrix 0.001
				  (unit-matrix (length q))))
   (base-torque? nil)
   (ddq-coeff
    (make-matrix (+ (length all-links) (if base-torque? 6 0))
		 (+ (length all-links) (if base-torque? 6 0))))
   (b (instantiate float-vector (+ (length all-links) (if base-torque? 6 0))))
   ddq#
   )
  (while (< (setq sim-time-buf (+ sim-time-buf sim-time-step))
	    sim-time-max)
    (calc-ddq-coeff-matrix
     :robot robot :q q :dq dq
     :all-links all-links
     :root-spacial-velocity root-spacial-velocity
     :root-angular-velocity root-angular-velocity
     :ddq-coeff ddq-coeff :b b :base-torque? base-torque?)
    (send *viewer* :draw-objects)
    (setq ddq# (pseudo-inverse ddq-coeff))
    (if ddq# (setq ddq (transform ddq# (v- (v- tau b) (transform friction-matrix dq)))))
    ;; dev check
    (setq
     ddq
     (map float-vector
	  #'(lambda (ddq l)
	      (cond
	       ((> (abs ddq) (* sim-time-step 10000))
		(format t " ---- too large! ~A~%" (send l :name))
		(* (if (plusp ddq) +1 -1) (* sim-time-step 10000)))
	       (t ddq)))
	  ddq all-links))
    ;;
    (setq dq (v+ dq (scale sim-time-step ddq)))
    (setq q (v+ q (scale (rad2deg sim-time-step) dq)))
    (format t "  (~A)  ddq=~A~%" sim-time-buf ddq)
    ))

