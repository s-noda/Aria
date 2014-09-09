#!/usr/bin/env roseus

;; dx = Ax+Bu
;; y  = Cx+Du
;; (x^, y^) E simulator

(defclass observed-state-feedback
  :super object
  :slots (x dx y u x^ dx^ y^ v
	    A B C D L F
	    timer))
(defmethod observed-state-feedback
  (:init
   (&rest
    args
    &key
    (x (instantiate float-vector 4))
    (dx (copy-seq x))
    (y (instantiate float-vector 2))
    (u (instantiate float-vector 2))
    (v (copy-seq u))
    (x^ (copy-seq x))
    (dx^ (copy-seq x))
    (y^ (copy-seq y))
    (A (unit-matrix (length x)))
    (B (make-matrix (length x) (length u)))
    (C (make-matrix (length y) (length x)))
    (D (make-matrix (length y) (length u)))
    (L (make-matrix (length x) (length y)))
    (F (transpose L))
    &allow-other-keys
    )
   (mapcar
    #'(lambda (sym)
	(let* ((key (read-from-string (format nil ":~A" (car sym)))))
	  ;; (print key)
	  ;; (print (eval (print (car sym))))
	  (send self key (or (eval (car sym)) (cdr sym)))))
    (send self :slots))
   )
  (:update
   (&key
    (tm (if timer (send timer :stop) 0))
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
    )
   (send self :u u)
   (send self :dx (setq dx (v+ (transform A x) (transform B u))))
   (send self :x (setq x (v+ x (scale tm dx))))
   (send self :y (setq y (v+ (transform C x) (transform D u))))
   (send self :dx^ (setq dx^ (v+ (transform (m- A (m* L C)) x^)
				 (v+ (transform B u)
				     (transform L (v- y (transform D u)))))))
   (send self :x^ (setq x^ (v+ x^ (scale tm dx^))))
   (send self :y^ (v+ (transform D u) (transform C x^)))
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

