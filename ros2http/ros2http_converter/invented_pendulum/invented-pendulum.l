
(require "observed-state-feedback.l")

(defclass invented-pendulm
  :super observed-state-feedback
  :slots (mm m r I g ur ux use-torso)
  )
(defmethod invented-pendulm
  (:init
   (&rest
    args
    &key
    (use-torso t)
    (x (instantiate float-vector 4))
    (y (instantiate float-vector 2))
    (u (instantiate float-vector (if use-torso 2 1)))
    (maxu (fill (copy-seq u) 10.0))
    (minu (fill (copy-seq u) -10.0))
    (mm 5.0)
    (m 4.0)
    (r 1.)
    (I 4)
    (g 9.8)
    (ur 3.0)
    (ux 3.0)
    &allow-other-keys)
   (mapcar
    #'(lambda (sym)
	(let* ((key (read-from-string (format nil ":~A" sym))))
	  (send self key (eval sym))))
    '(mm m r I g ur ux use-torso))
   (send-super* :init
		:x x :y y :u u
		:W (make-matrix 4 4
				'((2 0 0 0)
				  (0 2 0 0)
				  (0 0 0.01 0)
				  (0 0 0 0.01)
				  ))
		args)
   )
  (:A (&rest args)
      (let* ((q (aref x 1))
	     (dq (aref x 3))
	     (sq/q (if (< (abs q) 1e-6) 1.0 (/ (sin q) q)))
	     ;; (/ (sin q) (if (< (abs q) 1e-6) 1.0 q)))
	     )
	(m* (pseudo-inverse
	     (make-matrix
	      4 4
	      (list (list 1 0 0 0)
		    (list 0 1 0 0)
		    (list 0 0 (+ mm m) (* 1 m r (cos q)))
		    (list 0 0 (* m r (cos q)) (+ (* 1 I) (* 1 m r r))))))
	    (scale-matrix
	     -1
	     (make-matrix
	      4 4
	      (list (list 0 0 -1 0)
		    (list 0 0 0 -1)
		    (list 0 (* -1 m r sq/q dq dq) (* 1 ux) 0)
		    (list 0 (* -1 m r g sq/q) 0 (* 1 ur))))))))
  (:B (&rest args)
      (m* (pseudo-inverse
	   (make-matrix
	    4 4
	    (list (list 1 0 0 0)
		  (list 0 1 0 0)
		  (list 0 0 (+ mm m) (* 1 m r (cos (aref x 1))))
		  (list 0 0 (* m r (cos (aref x 1))) (+ (* 1 I) (* 1 m r r))))))
	  (if (not use-torso)
	      (make-matrix
	       4 1
	       (list (list 0)
		     (list 0)
		     (list (* +1 r))
		     (list 0)))
	    (make-matrix
	     4 2
	     (list (list 0 0)
		   (list 0 0)
		   (list (* +1 r) 0)
		   (list 0 +1))))
	  ))
  (:C (&rest args)
      (make-matrix 2 4 (list (list 1 0 0 0) (list 0 1 0 0))))
  )


(defun demo-invented-pendulum
  nil
  (bench
   (defvar *ip* (instance invented-pendulm :init))
   (send *ip* :riccati-l)
   (send *ip* :riccati-f)
   (send *ip* :check-f)
   (send *ip* :simulation :graph t :stop 1000 :debug? t)))
