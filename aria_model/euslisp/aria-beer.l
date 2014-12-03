
(ros::load-ros-manifest "geometry_msgs")
(ros::roseus "aria_beer")

(require "aria-interface.l")

(aria-init)

(send *robot* :reset-manip-pose)
(defvar *beer-end-coords*
  (send *robot*
	:put
	:beer-end-coords
	(make-cascoords
	 :name :beer-end-coords
	 :coords (make-coords :pos (send *robot* :rarm :end-coords :worldpos))
	 :parent (send *robot* :rarm :end-coords :parent))))

(defvar *beer-zero-coords* (make-coords))
(defvar *beer-zero-coords-calib-flag* t)
(defvar *beer-obj* (make-cylinder 80 200))
(send *beer-obj* :set-color #F(1 1 0))
(objects (list *robot* *beer-obj*))

(defun pose-callback
  (msg)
  (let* ((qv (float-vector
	      (send (send (send msg :pose) :orientation) :w)
	      (send (send (send msg :pose) :orientation) :x)
	      (send (send (send msg :pose) :orientation) :y)
	      (send (send (send msg :pose) :orientation) :z)))
	 (m (quaternion2matrix qv)))
    (cond
     (*beer-zero-coords-calib-flag*
      (setq *beer-zero-coords-calib-flag* nil)
      (setq *beer-zero-coords* (make-coords :rot (transpose m)))
      ))
    (send *beer-obj* :newcoords
	  (send *beer-zero-coords*
		:transformation
		(make-coords :rot (transpose m))))
    (send *robot* :inverse-kinematics
	  (send *beer-obj* :copy-worldcoords)
	  :move-target *beer-end-coords*
	  :link-list (send *robot* :link-list (send *robot* :rarm :end-coords :parent)
			   (send *robot* :rarm :root-link))
	  :rotation-axis t
	  :translation-axis nil
	  :stop 10
	  :revert-if-fail nil
	  :debug-view nil)
    (send *beer-obj* :newcoords
	  (send *beer-end-coords* :copy-worldcoords))
    ))

(defun calib-callback
  (msg) (setq *beer-zero-coords-calib-flag* t))

(ros::subscribe "/android_pose/poseStamped"
		geometry_msgs::PoseStamped
		#'pose-callback)
(ros::subscribe "/aria_beer/request/calib"
		std_msgs::empty
		#'calib-callback)


(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 (send *viewer* :draw-objects)
 (x::window-main-one)
 )

