#!/usr/bin/env roseus

(require "../float2string/euslisp/float2string.l")

(ros::roseus "sample_json_publihser")
(ros::roseus-add-msgs "std_msgs")

(ros::advertise
 "/ros2http/http_listener/json_string"
 std_msgs::String)

(ros::advertise
 "/ros2http/socket_listener/json_string"
 std_msgs::String)

(defvar *sensor-topics-name*
  (mapcar
   #'(lambda (str)
       (ros::advertise
	(format nil "/motorstates/~A/array" str)
	std_msgs::Float32MultiArray)
       (format nil "/motorstates/~A/array" str)
       )
   (list "position" "velocity" "temperature" "torque" "voltage")))

(defun string2float-vector
  (str
   &key
   (joint-cnt 30)
   (name-list *sensor-topics-name*)
   (float-list (mapcar #'(lambda (hoge) (instantiate float-vector joint-cnt))
		       name-list))
   (float-size (/ (length str) (* joint-cnt (length name-list) 2)))
   (float-buf #F(0))
   (pos 0)
   )
  (cond
   ((eq float-size 4)
    (dotimes (joint-id joint-cnt)
      (dotimes (name-id (length name-list))
	(string2float (subseq str pos (setq pos (+ pos (* 2 float-size))))
		      float-buf)
	;; (print float-buf)
	(setf (aref (nth name-id float-list) joint-id)
	      (aref float-buf 0))
	;;(dotimes (i (length str-buf))
	;;(setf (aref str-buf i)
	;;(+ (* (aref str pos) 16) (aref str (incf pos)))))
	))
    float-list))
  )

(defun float-vector2string
  (fv
   &key
   (joint-cnt 30)
   (str-buf
    (coerce (make-list (* joint-cnt 4 2) :initial-element #\0) string))
   )
  (dotimes (i (length fv))
    (float2string (aref fv i) str-buf (* i 4 2)))
  (if (< (length fv) joint-cnt)
      (setf (aref str-buf (* (length fv) 4 2)) #\0))
  str-buf)

(defvar *response-string*)
(defvar *response-float-vector*)
(defvar *response*)
(defvar *response-exist*)
(defun response-callback
  (msg)
  (setq *response* (send msg :data))
  ;; (print 'response)
  (setq *response-string* (parse-result (send msg :data)))
  ;; (format t "reponse: ~A~%" *response-string*)
  (setq *response-float-vector*
	(string2float-vector *response-string*))
  (setq *response-exist* *response-float-vector*)
  (cond
   (*response-exist*
    (mapcar
     #'(lambda (topic vec)
	 ;; (format t " publish ~A as ~A~%" topic vec)
	 (ros::publish
	  topic
	  (instance std_msgs::Float32MultiArray
		    :init
		    :data vec)))
     *sensor-topics-name* *response-float-vector*))
   (t (format t " response skip ~A~%" *response*)
      ))
  )

(ros::subscribe
 "/ros2http/socket_listener/reponse"
 std_msgs::String
 #'response-callback 1)

(defun parse-result
  (str)
  (let* ((pp (piped-fork (format nil "echo ~A | sed \"s/\:\\|\,/\\n/g\"" str)))
	 buf
	 (target "result")
	 (fin-flag nil))
    (while (setq buf (read-line pp nil))
      (cond
       (fin-flag (return-from nil nil))
       ((substringp target buf) (setq fin-flag t))))
    (close pp)
    buf))

(defun call-function
  (&key (func "getVersion")
	(param "[0]")
	(id 1)
	(spin? t))
  ;; (print 'call)
  (ros::publish
   "/ros2http/socket_listener/json_string"
   (instance std_msgs::String
	     :init
	     :data
	     ;;(print
	     (format nil "{\"method\":\"~A\",\"params\":~A,\"id\":~A}"
		     func param id)))
  ;;)
  (setq *response* nil)
  (if spin?
      (dotimes (i 100)
	(if *response* (return-from nil nil))
	(ros::sleep)
	(ros::spin-once)))
  *response*
  )

(defun sensor-feedback-loop
  (&key (hz 10))
  (ros::rate hz)
  (let ((time-step (/ 1.0 hz)))
    (do-until-key
     ;;(let* ((timer (instance mtimer :init)))
     (bench
      (if (not (ros::ok)) (return-from nil nil))
      (call-function :func "getValues" :spin? nil)
      (ros::sleep)
      ;;(unix:usleep (* 1000 5))
      (ros::spin-once)
      ;;(print (send timer :stop))
      ))))


#|

(ros::publish
 "/ros2http/socket_listener/json_string"
 (instance std_msgs::String
	   :init
	   :data "{\"method\":\"getVersion\",\"params\":[3],\"id\":\"1\"}"))

(ros::publish
 "/ros2http/socket_listener/json_string"
 (instance std_msgs::String
	   :init
	   :data "{\"method\":\"getValue\",\"params\":[3],\"id\":\"1\"}"))

(do-until-key
 (ros::publish
  "/ros2http/socket_listener/json_string"
  (instance std_msgs::String
	    :init
	    :data
	    (format nil "{\"method\":\"getValue\",\"params\":[~A],\"id\":\"1\"}"
		    (random 10))))
 (unix:usleep (* 10 1000))
 )

"0000000000000000000000000000000000000000ec609bbf000000000000d84100000000000000000082a63b000000000000d841000000000000000080bc65bd000000000000dc4100000000000000003d0624c0000000000000d8410000000000000000406163bd000000000000d841000000000000000080f738bd000000000000d841000000000000000010b7f93e000000000000d441000000000000000064e208bf000000000000dc410000000000000000b013793e000000000000dc41000000000000000070ed2a3e000000000000dc410000000000000000c04c62be000000000000d8410000000000000000004a873d000000000000dc4100000000000000008486193f000000000000d841000000000000000040b3063d000000000000d4410000000000000000cac6b73f000000000000dc410000000000000000c0893abd000000000000e0410000000000000000009697bc000000000000e04100000000000000004c69683f000000000000d4410000000000000000d0fbafbe000000000000d8410000000000000000a0b0f5be000000000000d841000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000090950bbf00000000000000000000000000000000"


3f358e3f
8e3f3f35
3f8e353f
3f358e3f

0000803f00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000