#!/usr/bin/env roseus

;; dx = Ax+Bu
;; y  = Cx+Du
;; (x^, y^) E simulator

(defclass observed-state-feedback
  :super object
  :slots (x dx y u x^ dx^ y^ v maxu minu
	    A B C D L F W
	    timer dt))
(defmethod observed-state-feedback
  (:init
   (&rest
    args
    &key
    (x (instantiate float-vector 4))
    (dx (copy-seq x))
    (y (instantiate float-vector 2))
    (u (instantiate float-vector 2))
    (maxu (fill (copy-seq u) 10))
    (minu (fill (copy-seq u) -10))
    (v (copy-seq u))
    (x^ (copy-seq x))
    (dx^ (copy-seq x))
    (y^ (copy-seq y))
    (A (unit-matrix (length x)))
    (B (make-matrix (length x) (length u)))
    (C (make-matrix (length y) (length x)))
    (W (unit-matrix (length x)))
    (D (make-matrix (length y) (length u)))
    (L (make-matrix (length x) (length y)))
    (F (make-matrix (length u) (length x)))
    &allow-other-keys
    )
   (mapcar
    #'(lambda (sym)
	(let* ((key (read-from-string (format nil ":~A" (car sym)))))
	  (send self key (or (eval (car sym)) (cdr sym)))))
    (send self :slots))
   )
  (:update
   (&key
    (tm (setq dt (if timer (send timer :stop) 0)))
    (F (send self :F))
    (x^ (send self :x^))
    (v (send self :v))
    (u (v- v (transform F x^)))
    ;;
    (A (send self :A))
    (x (send self :x))
    (B (send self :B))
    (C (send self :C))
    (dx (send self :dx))
    (D (send self :D))
    (L (send self :L))
    (dx^ (send self :dx^))
    (y (send self :y))
    (y^ (send self :y^))
    ;;
    (ys)
    )
   (setq
    u
    (map float-vector
	 #'(lambda (min u max)
	     (let ((u_ (min max (max min u))))
	       (if (or (< u min) (> u max))
		   (format t " [:update] warn u E/ (~A ~A) -> ~A~%" min max u_))
	       u_))
	 minu u maxu))
   ;; (setq u (vmax minu (vmin maxu u)))
   (send self :u u)
   (send self :dx (setq dx (v+ (transform A x) (transform B u))))
   (send self :x (setq x (v+ x (scale tm dx))))
   (send self :y (setq y (or ys (v+ (transform C x) (transform D u)))))
   (send self :dx^ (setq dx^ (v+ (transform (m- A (m* L C)) x^)
				 (v+ (transform B u)
				     (transform L (v- y (transform D u)))))))
   (send self :x^ (setq x^ (v+ x^ (scale tm dx^))))
   (send self :y^ (v+ (transform D u) (transform C x^)))
   (send self :timer (instance mtimer :init))
   (v- v (transform F x^))
   )
  (:simulation
   (&key (stop 300)
	 (step 0.01)
	 ret
	 (return-callback nil)
	 ;;#'(lambda (id &rest args) (send self :x)))
	 (graph nil)
	 graph-buf
	 (graph-callback
	  #'(lambda (id &rest args)
	      (list (cons :tm id)
		    ;;(cons :u1 (aref (send self :u) 0))
		    ;;(cons :u2 (aref (send self :u) 1))
		    (cons :x (aref (send self :x) 0))
		    (cons :q (aref (send self :x) 1))
		    ;;(cons :dx (aref (send self :x) 2))
		    ;;(cons :dq (aref (send self :x) 3))
		    )))
	 (init (progn
		 (send self :x
		       (map float-vector #'(lambda (a) (- (* 2 (random 0.1)) 0.1)) x))
		 (send self :y (scale 0 y))
		 (send self :y^ (scale 0 y))))
	 (debug? nil)
	 )
   (send self :x^ (send self :x))
   (dotimes (i stop)
     (if return-callback (push (funcall return-callback i) ret))
     (if graph (push (funcall graph-callback i) graph-buf))
     (if debug? (format t " x --> ~A~%" (send self :x)))
     (send self :update :tm step))
   (format t "[simulation in ~A sec]~% x=~A~% dx=~A~% x_dif=~A~%"
	   (* stop step)
	   (send self :x) (send self :dx) (v- (send self :x) (send self :x^)))
   (cond
    (graph
     (require "euslib/demo/s-noda/util/graph-sample.l")
     (setq
      graph
      (create-graph
       "simulation graph"
       :name-list (mapcar #'(lambda (d) (format nil "~A" (car d)))
			  (remove-if #'(lambda (d) (find :tm d)) (car graph-buf)))
       :data-list (mapcar #'(lambda (name)
			      (mapcar #'(lambda (data)
					  (float-vector (cdr (assoc :tm data))
							(cdr (assoc name data))))
				      graph-buf))
			  (remove :tm (mapcar #'car (car graph-buf))))))
     (send graph :fit-draw)))
   ret
   )
  (:solve-riccati
   (&key
    (A (send self :A))
    (Q (m* (transpose (send self :W)) (send self :W)))
    (B (send self :B))
    (R (unit-matrix (length u)))
    (At (transpose A))
    (Bt (transpose B))
    (max 500000)
    (conv-thre-rate 1e-6)
    (max-conv-thre 1e-6)
    (min-conv-thre 1e-12)
    (conv-thre)
    (conv-flag nil)
    (P (make-matrix (length x) (length x)))
    (dP) ;; (copy-object P))
    (gain -1e-2)
    (debug? nil)
    )
   (dotimes (i max)
     (setq dP
	   (m+ (m* At P)
	       (m+ (m* P A)
		   (m- Q
		       (m* P (m* B (m* R (m* Bt P))))))))
     (cond
      ((null conv-thre)
       (setq conv-thre
	     (min max-conv-thre
		  (max min-conv-thre
		       (* conv-thre-rate (norm (send dP :get-val 'entity))))))
       (format t " :solve-riccati thre ~A~%" conv-thre)))
     (if debug? (print (norm (send dp :get-val 'entity))))
     (if (< (norm (send dp :get-val 'entity)) conv-thre)
	 (return-from nil (setq conv-flag t)))
     (setq P (m- P (scale-matrix gain dP)))
     )
   (if (not conv-flag) (format t " :solve-riccati failed ~A > ~A~%"
			       (norm (send dp :get-val 'entity))
			       conv-thre
			       ))
   P)
  (:riccati-l
   nil
   (let* ((P (send self :solve-riccati
		   :A (transpose (send self :A))
		   :B (transpose (send self :C))
		   :R (unit-matrix (length y))
		   )))
     (send self :L (transpose (m* (send self :C) P)))
     ))
  (:riccati-f
   nil
   (let* ((P (send self :solve-riccati)))
     (send self :F (m* (transpose (send self :B)) P))
     ;; (send self :L (transpose (send self :F)))
     )
   ;; (let* ((rc (instance riccati-equation :init
   ;; 			(send self :A)
   ;; 			(send self :B)
   ;; 			;; (send self :C)
   ;; 			(send self :W)
   ;; 			1 1e-6)))
   ;;   (send rc :solve)
   ;;   (send self :F
   ;; 	   (m* (transpose (send self :B)) (send rc :get-val 'P)))
   ;;   rc
   ;; (send self :L (transpose (send self :F)))
   )
  (:check-f
   (&key (m (m- (send self :A) (m* (send self :B) (send self :F))))
	 (eigen (instantiate float-vector (send m :get-val 'dim0))))
   (require "package://eus_eigen/euslisp/eus_eigen.l")
   (calc-eigen :mat m :peigenval eigen)
   (format t " [:check-f]~%")
   (format-array m)
   (format t "  ---> ")
   (print eigen)
   (if (find-if #'plusp eigen) (format t "   !!!!! dangerous !!!!~%"))
   )
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

