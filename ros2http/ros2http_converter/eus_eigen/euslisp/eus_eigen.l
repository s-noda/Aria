#!/usr/bin/env roseus

(defvar *float2string-plugin*
  (let* ((in (read-line (piped-fork "rospack find ros2http")))
	 (path (format
		nil
		"~A/ros2http_converter/eus_eigen/lib/libeus_eigen.so"
		in)))
    (if (not (probe-file path))
	(setq path
	      (read-line (piped-fork "locate libeus_eigen.so") nil)))
    (cond
     ((null path)
      (print 'libfloat2string.so-missing)
      (exit -1))
     (t (load-foreign path)))))

(defforeign _calc-eigen
  *float2string-plugin*
  "calc_eigen"
  (:integer :string :string :string :string :string)
  :integer)

(defun calc-eigen
  (&key
   (mat (diagonal (random-vector 1.0)))
   (n (send mat :get-val 'dim0))
   (peigenval (instantiate float-vector n))
   (neigenval (instantiate float-vector n))
   (peigenvec (instantiate float-vector (* n n)))
   (neigenvec (instantiate float-vector (* n n)))
   )
  (_calc-eigen n (send mat :get-val 'entity) peigenval neigenval peigenvec neigenvec)
  )

(defun calc-eigen-test
  (&key
   (mat (diagonal (random-vector 1.0)))
   )
  (calc-eigen :mat mat)
  (format t "[calc-eigen-test]~%")
  (format t "   target~%")
  (format-array mat)
  (format t "   eigen~% ~A~%" peigenval)
  )
