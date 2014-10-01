;; create window to set joint-angle
(defclass robot-joint-angle-slider
  :super x::panel
  :slots (slider-list
	  joint-list)
  )

(defmethod robot-joint-angle-slider
  (:create
   (jlist &optional (name-list (send-all jlist :name)) &rest args)
   (send-super* :create :width 400 :height (* (length jlist) 30)
		:font x::font-courb12 args)
   (setq joint-list jlist)
   (setq slider-list
	 (mapcar #'(lambda (joint name)
		     (send-super :create-item x::slider-item
				 (let ((name (format nil "~A" name)))
				   (while (< (length name) 15)
				     (setq name
					   (concatenate string name " ")))
				   name)
				 self :set-robot-pose
				 :min (max -120 (send joint :min-angle))
				 :max (min 120 (send joint :max-angle))
				 ;;:value (send joint :joint-angle)
				 ;; can't set value before instantiate v
				 ))
		 joint-list name-list))
   (labels ( (set-value (joint-list slider-list)
			(if (or (null joint-list) (null slider-list))
			    t
			  (progn
			    (send (car slider-list) :value
				  (send (car joint-list) :joint-angle))
			    (set-value (cdr joint-list) (cdr slider-list))
			    )
			  )
			) )
     (set-value joint-list slider-list))
   self)
  (:set-robot-pose
   (obj val)
   (labels ( (set-value (joint-list slider-list)
			(if (or (null joint-list) (null slider-list))
			    t
			  (progn
			    (send (car slider-list) :value
				  (send (car joint-list) :joint-angle))
			    (set-value (cdr joint-list) (cdr slider-list))
			    )
			  )
			) )
     (set-value joint-list slider-list))
;   (send (send *robot* :joint (string-right-trim " " (send obj :name)))
;	 :joint-angle val)
   (send (cdr (assoc obj (mapcar #'cons slider-list joint-list)))
	 :joint-angle val)
   (if (boundp '*pickview*) (send *pickview* :draw-objects))
   (if (boundp '*irtviewer*) (send *irtviewer* :draw-objects))
   )
  )


;; string-split
(defun string-split (str split)
  (labels ( (string-split-recursive
	     (close str split)
	     (cond
	      ( (null str) (list (concatenate string (reverse close))) )
	      ( (= (car str) split)
		(cons (concatenate string (reverse close))
		      (string-split-recursive nil (cdr str) split)) )
	      (t (string-split-recursive
		  (cons (car str) close) (cdr str) split)))
	      )
	     )
    (string-split-recursive nil (concatenate cons str) split)))


;; rapper to create window
(defun robot-pose-creater (joint-list)
  (let* ( (get-name #'(lambda (joint) (send joint :name)))
	  (name-list (mapcar get-name joint-list))
	  (get-class #'(lambda (joint)
			 (let* ( (name (funcall get-name joint))
				 (split (string-split name #\_)) )
			   (cadr (reverse split)))))
	  (class-list-duplicate (mapcar get-class joint-list))
	  (class-list
	   (remove-duplicates class-list-duplicate :test #'equal)) )
    (labels ( (get-class-list
	       (joint-list clss-string)
	       (cond
		( (null joint-list) nil )
		( (equal (funcall get-class (car joint-list)) clss-string)
		  (cons (car joint-list)
			(get-class-list (cdr joint-list) clss-string)) )
		(t (get-class-list (cdr joint-list) clss-string))
		)
	       ) )
	      (dolist (clss-string class-list)
		(instance robot-pose :create
			  (get-class-list joint-list clss-string)))
	      )
    )
  )

(defvar *robot*)
(defvar *joint-list*)

(cond
 (*robot*
  (setq *joint-list* (or *joint-list* (send *robot* :joint-list)))
  (setq
   *pose-slider*
   (mapcar
    #'(lambda (k)
	(instance robot-joint-angle-slider :create
		  (send *robot* k :joint-list)
		  (mapcar
		   #'(lambda (name)
		       (read-from-string
			(concatenate
			 string
			 ":"
			 (subseq (format nil "~A" k) 1)
			 "-"
			 (subseq (format nil "~A" name) 1))))
		   (send-all (send *robot* k :joint-list) :name))))
    '(:torso :rarm :larm :head)))))

#|
(instance robot-pose :create
	  *joint-list*
	  (flatten
	   (mapcar
	    #'(lambda (k)
		(mapcar
		 #'(lambda (name)
		     (read-from-string
		      (concatenate
		       string
		       ":"
		       (subseq (format nil "~A" k) 1)
		       "-"
		       (subseq (format nil "~A" name) 1))))
		 (send-all (send *robot* k :joint-list) :name)))
	    '(:torso :rarm :larm :rleg :lleg :head))))

#f(14.07 1.38468 -1.65682 8.43754 -5.17532 -0.508997 0.0 44.0 1.41464 -1.74905 8.4752 -5.13237 -0.517853 0.0 0.248154 1.82496 -43.0 -19.75 -65.6283 -9.0 -50.96 -105.86 42.6758 -22.6135 7.86434 10.53 -57.62 -9.0 -37.621 -90.79 63.0 -28.8 -3.6 0.0)


#f(21.0406 -5.09109 -11.7347 31.7569 -18.7494 8.68061 0.0 9.34557 11.271 -30.8032 113.822 -7.67912 -5.54066 0.0 -4.87798 20.7335 -23.9982 -28.0 -152.82 23.96 4.89168 -42.9039 0.0 0.0 -9.62527 0.0 50.8373 6.4249 2.07223 -27.7705 0.0 0.0 -10.0 0.0)