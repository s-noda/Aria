#!/usr/bin/env roseus

(defvar *float2string-plugin*
  ;; (load-foreign "../lib/libfloat2string.so"))
  (let* ((in (read-line (piped-fork "rospack find ros2http")))
	 (path (format
		nil
		"~A/ros2http_converter/float2string/lib/libfloat2string.so"
		in)))
    (if (not (probe-file path))
	(setq path
	      (read-line (piped-fork "locate libfloat2string.so") nil)))
    (cond
     ((null path)
      (print 'libfloat2string.so-missing)
      (exit -1))
     (t (load-foreign path)))))

(defforeign hello-world-test
  *float2string-plugin*
  "hello_world_test"
  ()
  :integer)

(defforeign float2string
  *float2string-plugin*
  "float_to_string_for_eus"
  (:double :string :integer)
  :integer)

(defforeign string2float
  *float2string-plugin*
  "string_to_float_for_eus"
  (:string :string)
  :integer)

(defun float2string-test
  (&key
   (org (random 1.0))
   (str-buf "00")
   (flo-buf #F(0))
   )
  (float2string org str-buf 0)
  (string2float str-buf flo-buf)
  (format t "[float2string-test]~%")
  (format t " org = ~Af~%" org)
  (format t " str = ~As~%" str-buf)
  (format t " flo = ~Af~%" (aref flo-buf 0))
  (format t " dif = ~Af~%" (- org (aref flo-buf 0)))
  )
