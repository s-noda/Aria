#!/usr/bin/env roseus

(require "sample_json_publisher.l")
(require "euslib/demo/sasabuchi/sasabody/model/sasabody.l")

(defvar *robot* (sasabody))
(objects *robot*)
(send *irtviewer* :change-background #F(0.9 0.9 0.9))
(send *irtviewer* :draw-objects)

(send-all (send *robot* :joint-list) :min-angle -10000)
(send-all (send *robot* :joint-list) :max-angle 10000)

(setq *joint-list*
  (mapcar
   #'(lambda (name)
       (find-if #'(lambda (j) (string-equal name (send j :name)))
		(send *robot* :joint-list)))
   (list
    "arm_r_joint1" "arm_r_joint2" "arm_r_joint3" "arm_r_joint4"
    "arm_r_joint5" "arm_r_joint6" "arm_r_joint7" ;;"arm_l_hand1"
    "arm_l_joint1" "arm_l_joint2" "arm_l_joint3" "arm_l_joint4"
    "arm_l_joint5" "arm_l_joint6" "arm_l_joint7" ;;"arm_r_hand1"
    "neck1" "neck2" "neck3"
    "hip2" "hip3" "body"
    )))
(setq *mirror-bin*
  (list
   1 1 1 -1 -1 -1 -1
   1 1 1 -1 +1 -1 +1
   1 1 1
   1 0.5 -0.5))

(defun gained-torque
  (tor)
  (let* ((val (abs tor)))
    (* tor 8)))

(defun sensor-feedback-loop
  (&key (hz 100))
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
	 (cond
	  (*response-float-vector*
	   (map cons
		#'(lambda (pos mir J)
		    (send j :joint-angle (* mir (rad2deg pos))))
		(nth 0 *response-float-vector*)
		*mirror-bin*
		*joint-list*)
	   (send *viewer* :draw-objects)
	   (x::window-main-one)
	   (send *robot* :calc-torque
		 :target-coords (list (send *robot* :rarm :end-coords)
				      (send *robot* :larm :end-coords))
		 :force-list (list #F(0 0 0) #F(0 0 0))
		 :moment-list (list #F(0 0 0) #F(0 0 0))
		 )))
	 (print (send *robot* :rarm :shoulder-r :joint-torque))
	 (call-function :func "setTorques2"
			:param
			(format nil "[~A]"
				(float-vector2string
				 (map float-vector
				      #'gained-torque
				      (map float-vector
					   #'*
					   (scale -1 #F(1 1 1 1 1 0 0))
					   (send-all *joint-list* :joint-torque)
					   *mirror-bin*
					   ))))
			:spin? nil)
	 ;; (call-function :func "getValues" :spin? nil)
	 (format t "[~A]~%" (if *response-exist* "receive" "skip"))
	 (format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	 (format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	 (incf cnt)
	 (if *response-exist* (incf pub-cnt))
	 (setq *response-exist* nil)
	 )))))))

#|
(defun sensor-feedback-loop
  (&key (hz 30))
  (ros::rate hz)
  (let ((time-step (/ 1.0 hz))
	(cnt 0)
	(pub-cnt 0)
	(time 0)
	)
    (do-until-key
     ;;(let* ((timer (instance mtimer :init)))
     (setq
      time
      (+
       time
       (bench2
	(if (not (ros::ok)) (return-from nil nil))
	(send *robot* :calc-torque
	      :target-coords (list (send *robot* :rarm :end-coords)
				   (send *robot* :larm :end-coords))
	      :force-list (list #F(0 0 0) #F(0 0 0))
	      :moment-list (list #F(0 0 0) #F(0 0 0))
	      )
	(call-function :func "setTorques2"
		       :param
		       (format nil "[~A]"
			       (float-vector2string
				(map float-vector
				     #'*
				     (scale -7 #F(0 1 1 1 0 0))
				     (send-all *joint-list* :joint-torque)
				     *mirror-bin*
				     )))
		       :spin? nil)
	;; (call-function :func "getValues" :spin? nil)
	(ros::spin-once)
	(ros::sleep)
	(map cons
	     #'(lambda (pos mir J)
		 (send j :joint-angle (* mir (rad2deg pos))))
	     (nth 0 *response-float-vector*)
	     *mirror-bin*
	     *joint-list*)
	(send *viewer* :draw-objects)
	(x::window-main-one)
	(format t "[~A]~%" (if *response-exist* "receive" "skip"))
	(format t "  SUB: ~A/~A = ~A~%" cnt time (/ (* cnt 1.0) time))
	(format t "  PUB: ~A/~A = ~A~%" pub-cnt time (/ (* pub-cnt 1.0) time))
	(incf cnt)
	(if *response-exist* (incf pub-cnt))
	(setq *response-exist* nil)
	))))))
