#!/usr/bin/env roseus

(defvar *eigen-plugin*
  (labels
   ((library_search
     (str &key
	  (depth 0)
	  colon-pos lib-path)
     (format t "  [~A] target=" depth)
     (cond
      ((eq (length str) 0)
       (format t "~% euseigen_plugin_not_found in eus_eigen.l~%")
       (exit -1))
      ((and (setq colon-pos (or (position #\: str) (length str)))
	    (setq lib-path (subseq str 0 colon-pos))
	    (setq lib-path
		  (if (eq (aref lib-path (- (length lib-path) 1)) #\/)
		      (subseq lib-path 0 (- (length lib-path) 1))
		    lib-path))
	    (probe-file (setq lib-path
			      (print (format nil "~A/libeus_eigen.so" lib-path)))))
       (load-foreign lib-path))
      (t
       (library_search (subseq str (min (length str) (+ colon-pos 1)))
		       :depth (+ depth 1))))))
   (library_search (format nil "~A:~A/lib"
			   (unix:getenv "LD_LIBRARY_PATH")
			   (read-line (piped-fork "rospack find eus_eigen"))))))

(defforeign _calc-eigen
  *eigen-plugin*
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
   (n (send mat :get-val 'dim0))
   (peigenval (instantiate float-vector n))
   (neigenval (instantiate float-vector n))
   (peigenvec (instantiate float-vector (* n n)))
   (neigenvec (instantiate float-vector (* n n)))
   )
  (calc-eigen :mat mat :peigenval peigenval)
  (format t "[calc-eigen-test]~%")
  (format t "   target~%")
  (format-array mat)
  (format t "   eigen~% ~A~%" peigenval)
  )
