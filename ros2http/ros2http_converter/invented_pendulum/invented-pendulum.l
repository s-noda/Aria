
(require "observed-state-feedback.l")

(defclass invented-pendulm
  :super observed-state-feedback
  :slots (mm m r I g ur ux)
  )
(defmethod invented-pendulm
  (:init
   (&rest
    args
    &key
    (x (instantiate float-vector 4))
    (y (instantiate float-vector 2))
    (u (instantiate float-vector 2))
    (mm 5.0)
    (m 4.0)
    (r 0.5)
    (I 10)
    (g 9.8)
    (ur 0.1)
    (ux 0.1)
    &allow-other-keys)
   (send-super* :init args)
   (mapcar
    #'(lambda (sym)
	(let* ((key (read-from-string (format nil ":~A" (car sym)))))
	  (send self key (or (eval (car sym)) (cdr sym)))))
    (send self :slots))
   )
  (:A (&rest args)
      (m* (pseudo-inverse
	   (make-matrix
	    4 4
	    (list (list 1 0 0 0)
		  (list 0 1 0 0)
		  (list 0 0 (+ mm m) (* -1 m r))
		  (list 0 0 (* m r) (- I (* m r r))))))
	  (make-matrix
	   4 4
	   (list (list 0 0 1 0)
		 (list 0 (* m r g) 0 1)
		 (list 0 0 (* -1 ux) 0)
		 (list 0 0 0 (* -1 ur))))))
  (:B (&rest args)
      (m* (pseudo-inverse
	   (make-matrix
	    4 4
	    (list (list 1 0 0 0)
		  (list 0 1 0 0)
		  (list 0 0 (+ mm m) (* -1 m r))
		  (list 0 0 (* m r) (- I (* m r r))))))
	  (make-matrix
	   4 2
	   (list (list 0 0)
		 (list 0 0)
		 (list 1 0)
		 (list 0 1)))))
  (:C (&rest args)
      (make-matrix 2 4 (list (list 1 0 0 0) (list 0 1 0 0))))
  )

(defvar a (instance invented-pendulm :init))
;; (print (send a :update))