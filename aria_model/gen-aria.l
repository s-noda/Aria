
(defun replace-a-to-b
  (str a b)
  (let* ((sed (piped-fork (format nil "echo ~A | sed \"s/~A/~A/g\"" str a b)))
	 (ret (read-line sed)))
    (close sed)
    ret))

(defun parse-link.dat
  (&key (path "link.dat"))
  (let* ((in (open path :direction :input))
	 buf
	 key
	 val-list
	 ret)
    (while (setq buf (read-line in nil))
      (cond
       ((eq (aref buf 0) #\:)
	(if key (push (cons key (reverse val-list)) ret))
	(setq val-list nil)
	(setq key (read-from-string buf))
	)
       (t
	(push
	 (if (eq key :name)
	     (subseq
	      (map string #'char-downcase
		   (replace-a-to-b buf "\ " "_"))
	      (length "ric_torso_type_l_"))
	   (read-from-string buf))
	 val-list))))
    (if key (push (cons key (reverse val-list)) ret))
    (close in)
    ret))

(defvar *robot-data* (parse-link.dat))
(defvar *dae-list* (directory "dae"))
(defvar *dae-root* "package://aria_model/dae")

(defvar *link-definition*
  "<link name=\"~A\">
    <visual>
      <origin rpy=\"~A ~A ~A\" xyz=\"~A ~A ~A\"/>
      <geometry>
        ~A
        <!--<cylinder length=\"0.10\" radius=\"0.03\"/>-->
      </geometry>
    </visual>
    <collision>
      <origin rpy=\"~A ~A ~A\" xyz=\"~A ~A ~A\"/>
      <geometry>
        <cylinder length=\"0.10\" radius=\"0.03\"/>
      </geometry>
    </collision>
    <inertial>
      <origin rpy=\"~A ~A ~A\" xyz=\"~A ~A ~A\"/>
      <mass value=\"~A\"/>
      <inertia ixx=\"~A\" ixy=\"~A\" ixz=\"~A\" iyy=\"~A\" iyz=\"~A\" izz=\"~A\"/>
    </inertial>
  </link>")

(defvar *joint-definition*
  "<joint name=\"~A\" type=\"revolute\">
    <origin rpy=\"~A ~A ~A\" xyz=\"~A ~A ~A\"/>
    <axis xyz=\"~A ~A ~A\"/>
    <parent link=\"~A\"/>
    <child link=\"~A\"/>
    <limit effort=\"~A\" lower=\"~A\" upper=\"~A\" velocity=\"~A\"/>
    <dynamics damping=\"~A\" friction=\"~A\"/>
  </joint>")

(defvar *transmission-definition*
  "<transmission name=\"~A\">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name=\"~A\"/>
    <actuator name=\"~A\">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>")

(defun id-coords
  (&key (id 1)
	(data *robot-data*)
	(pos (position id (cdr (assoc :id data))))
	(name (if pos (nth pos (cdr (assoc :name data)))))
	(rpy (if pos
		 (list (nth pos (cdr (assoc :rx data)))
		       (nth pos (cdr (assoc :ry data)))
		       (nth pos (cdr (assoc :rz data))))))
	(xyz (if pos
		 (list (nth pos (cdr (assoc :px data)))
		       (nth pos (cdr (assoc :py data)))
		       (nth pos (cdr (assoc :pz data)))))))
  (if pos
      (make-coords :pos (coerce xyz float-vector)
		   :rpy (mapcar #'deg2rad (reverse rpy)))
    (make-coords)))

(defun gen-link-definition
  (&key
   (data *robot-data*)
   (id 1)
   (pos (position id (cdr (assoc :id data))))
   (parent-id (nth pos (cdr (assoc :parent data))))
   (name (format nil "~A_link" (nth pos (cdr (assoc :name data)))))
   (dae-name-candidates
    (mapcar
     #'(lambda (str)
	 (cons
	  (map string #'char-downcase (replace-a-to-b str "\ " "_"))
	  str))
     *dae-list*))
   (dae-name
    (let ((nm (nth pos (cdr (assoc :name data)))))
      (cdr (find-if
	    #'(lambda (a)
		;; (format t "~A vs ~A~%" nm (car a))
		(substringp nm (car a)))
	    dae-name-candidates))))
   (self-coords (id-coords :id id))
   (parent-coords (id-coords :id parent-id))
   (coords (make-coords))
   ;;(send parent-coords :transformation self-coords :local))
   (rpy (reverse (car (rpy-angle (send coords :worldrot)))))
   (xyz (coerce (send coords :worldpos) cons))
   (m   (nth pos (cdr (assoc :m data))))
   (c-from-joint
    (list (nth pos (cdr (assoc :cx data)))
	  (nth pos (cdr (assoc :cy data)))
	  (nth pos (cdr (assoc :cz data)))))
   (c-world
    (send (copy-object self-coords)
	  :transform
	  (make-coords :pos (coerce c-from-joint float-vector))
	  :local))
   (c-from-link
    (coerce (v- (send c-world :worldpos)
		(send self-coords :worldpos))
	    cons))
   (I   (list (nth pos (cdr (assoc :ixx data)))
	      (nth pos (cdr (assoc :ixy data)))
	      (nth pos (cdr (assoc :ixz data)))
	      (nth pos (cdr (assoc :iyy data)))
	      (nth pos (cdr (assoc :iyz data)))
	      (nth pos (cdr (assoc :izz data)))))
   )
  ;; print dae-name-candidates)
  (format t " ~A->~A = ~A~%"
	  (nth (or (position parent-id (cdr (assoc :id data))) id)
	       (cdr (assoc :name data)))
	  name
	  coords)
  (eval
   (append
    '(format nil *link-definition*)
    (flatten
     (list name
	   rpy
	   (mapcar #'(lambda (d) (* d 1e-3)) xyz)
	   (if dae-name
	       (format nil
		       "<mesh filename=\"~A/~A\" scale=\"0.01 0.01 0.01\" />"
		       *dae-root* dae-name)
	     "<cylinder length=\"0.10\" radius=\"0.03\"/>")
	   ;; '(0 0 1)
	   rpy
	   (mapcar #'(lambda (d) (* d 1e-3)) xyz)
	   '(0.0 0.0 0.0)
	   (mapcar #'(lambda (d) (* d 1e-3)) c-from-joint) ;; c-from-link)
	   (* 1e-3 m)
	   (mapcar #'(lambda (d) (* d 1e-6)) I)
	   )))))

(defun gen-joint-definition
  (&key
   (data *robot-data*)
   (id 2)
   (pos (position id (cdr (assoc :id data))))
   (name (format nil "~A" (nth pos (cdr (assoc :name data)))))
   (parent-id (nth pos (cdr (assoc :parent data))))
   (parent-pos (position parent-id (cdr (assoc :id data))))
   ;;
   (parent-coords (id-coords :id parent-id))
   ;; (b   (list (nth pos (cdr (assoc :bx data)))
   ;; 	      (nth pos (cdr (assoc :by data)))
   ;; 	      (nth pos (cdr (assoc :bz data)))))
   ;; (self-coords (make-coords
   ;; 		 :pos (v+ (send parent-coords :worldpos)
   ;; 			  (coerce b float-vector))
   ;; 		 :rot (copy-object (send parent-coords :worldrot))))
   (self-coords (id-coords :id id))
   (coords (send parent-coords :transformation self-coords :local))
   (rpy (reverse (car (rpy-angle (send coords :worldrot)))))
   (xyz (coerce (send coords :worldpos) cons))
   (jxyz xyz)
   (axis '(0 0 1))
   (parent-name (format nil "~A_link" (nth parent-pos (cdr (assoc :name data)))))
   (child-name (format nil "~A_link" (nth pos (cdr (assoc :name data)))))
   (effort 10)
   (lower -3.14)
   (upper 3.14)
   (velocity 1)
   (damping 0.7)
   (friction 0.0)
   )
  (eval
   (append
    '(format nil *joint-definition*)
    (flatten
     (list name
	   rpy
	   (mapcar #'(lambda (d) (* d 1e-3)) jxyz)
	   axis
	   parent-name
	   child-name
	   effort lower upper velocity
	   damping friction)))))

(defun gen-transmission-definition
  (&key
   (data *robot-data*)
   (id 2)
   (pos (position id (cdr (assoc :id data))))
   (name (nth pos (cdr (assoc :name data))))
   (tname (format nil "~A_trans" name))
   (jname (format nil "~A" name))
   (aname (format nil "~A_act" name))
   )
  (eval
   (append
    '(format nil *transmission-definition*)
    (list tname jname aname))))

(defun gen-urdf
  (&key
   (path "aria.urdf")
   (data *robot-data*)
   (id (cdr (assoc :id data)))
   (parent (cdr (assoc :parent data)))
   )
  (let* ((out (open path :direction :output))
	 )
    (format out "<?xml version=\"1.0\" ?>~%<robot name=\"aria\">~%")
    (format out "<link name=\"base\"></link>~%")
    (mapcar
     #'(lambda (i)
	 (format out "~A~%" (gen-link-definition :id i)))
     id)
    (mapcar
     #'(lambda (i)
	 (cond
	  ((find (nth (position i id) parent) id)
	   (format out "~A~%" (gen-joint-definition :id i)))
	  (t
	   (format out "<joint name=\"~A_fixed\" type=\"fixed\">
    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>
    <parent link=\"base\"/>
    <child link=\"~A_link\"/>
    <limit effort=\"50000\" lower=\"-3.01\" upper=\"3.01\" velocity=\"10000\"/>
  </joint>~%"
		   (nth (position i id) (cdr (assoc :name data)))
		   (nth (position i id) (cdr (assoc :name data)))))))
     id)
    ;; ssb
    (format out "~A~%"
	    "<joint name=\"eye_r_hori\" type=\"revolute\">
    <origin rpy=\"0.05 -0.419 -0.189\" xyz=\"0 0 0\"/>
    <axis xyz=\"0 1 0\"/>
    <parent link=\"neck3_link\"/>
    <child link=\"eyer_link\"/>
    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"1\"/>
    <dynamics damping=\"0.7\" friction=\"0.0\"/>
  </joint>
<link name=\"eyer_link\">
    <inertial>
      <mass value=\"0.0\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <inertia ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\"/>
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0.01 0.01 0.01\"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0 0 0\"/>
      </geometry>
    </collision>
  </link>
<joint name=\"eye_r_vert\" type=\"revolute\">
    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>
    <axis xyz=\"0 0 1\"/>
    <parent link=\"eyer_link\"/>
    <child link=\"eyer_end_link\"/>
    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"1\"/>
    <dynamics damping=\"0.7\" friction=\"0.0\"/>
  </joint>
<link name=\"eyer_end_link\">
    <inertial>
      <mass value=\"0.0\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <inertia ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\"/>
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0.1 0.1 0.1\"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0 0 0\"/>
      </geometry>
    </collision>
  </link>
<joint name=\"eye_l_hori\" type=\"revolute\">
    <origin rpy=\"-0.05 0.419 -0.189\" xyz=\"0 0 0\"/>
    <axis xyz=\"0 1 0\"/>
    <parent link=\"neck3_link\"/>
    <child link=\"eyel_link\"/>
    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"1\"/>
    <dynamics damping=\"0.7\" friction=\"0.0\"/>
  </joint>
<link name=\"eyel_link\">
    <inertial>
      <mass value=\"0.0\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <inertia ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\"/>
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0.01 0.01 0.01\"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0 0 0\"/>
      </geometry>
    </collision>
  </link>
<joint name=\"eye_l_vert\" type=\"revolute\">
    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>
    <axis xyz=\"0 0 1\"/>
    <parent link=\"eyel_link\"/>
    <child link=\"eyel_end_link\"/>
    <limit effort=\"10\" lower=\"-3.14\" upper=\"3.14\" velocity=\"1\"/>
    <dynamics damping=\"0.7\" friction=\"0.0\"/>
  </joint>
<link name=\"eyel_end_link\">
    <inertial>
      <mass value=\"0.0\"/>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <inertia ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\"/>
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0.1 0.1 0.1\"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
      <geometry>
        <box size=\"0 0 0\"/>
      </geometry>
    </collision>
  </link>")
    (format out "~A~%"
	    "<joint name=\"camera_joint\" type=\"fixed\">
      <origin xyz=\"0.1525 0.082 -0.005\" rpy=\"-1.57 0 0\"/>
      <!-- <origin xyz=\"0.0185 0.06 0\" rpy=\"-1.57 0 0\"/> -->
      <parent link=\"neck3_link\"/>
      <child link=\"camera_link\"/>
    </joint>
    <link name=\"camera_link\">
      <inertial>
	<mass value=\"0.200\" />
	<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />
	<inertia ixx=\"5.8083e-4\" ixy=\"0\" ixz=\"0\" iyy=\"3.0833e-5\" iyz=\"0\" izz=\"5.9083e-4\" />
      </inertial>
      <visual>
	<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />
	<geometry>
	  <!-- <box size=\"0.037 0.18 0.025\"/> -->
	  <mesh filename=\"package://hector_sensors_description/meshes/asus_camera/asus_camera_simple.dae\"/>
	</geometry>
      </visual>
      <collision>
	<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />
	<geometry>
	  <box size=\"0.035 0.185 0.025\"/>
	</geometry>
      </collision>
    </link>
    <joint name=\"camera_depth_joint\" type=\"fixed\">
      <origin xyz=\"0.0 0.049 0.0\" rpy=\"0 0 0\" />
      <parent link=\"camera_link\" />
      <child link=\"camera_depth_frame\"/>
    </joint>
    <link name=\"camera_depth_frame\"/>
    <joint name=\"camera_depth_optical_joint\" type=\"fixed\">
      <origin xyz=\"0 0 0\" rpy=\"-1.57 0.0 -1.57\" />
      <parent link=\"camera_depth_frame\" />
      <child link=\"camera_depth_optical_frame\"/>
    </joint>
    <link name=\"camera_depth_optical_frame\"/>
    <joint name=\"camera_rgb_joint\" type=\"fixed\">
      <origin xyz=\"0.0 0.022 0.0\" rpy=\"0 0 0\" />
      <parent link=\"camera_link\" />
      <child link=\"camera_rgb_frame\"/>
    </joint>ex
    <link name=\"camera_rgb_frame\"/>
    <joint name=\"camera_rgb_optical_joint\" type=\"fixed\">
      <origin xyz=\"0 0 0\" rpy=\"-1.57 0.0 -1.57\" />
      <parent link=\"camera_rgb_frame\" />
      <child link=\"camera_rgb_optical_frame\"/>
    </joint>
    <link name=\"camera_rgb_optical_frame\"/>")
    (mapcar
     #'(lambda (i)
	 (cond
	  ((find (nth (position i id) parent) id)
	   (format out "~A~%" (gen-transmission-definition :id i)))))
     id)
    (formate out "~A~%"
	     "<transmission name=\"eye_r_hori_trans\">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name=\"neck3\"/>
    <actuator name=\"eye_r_hori_act\">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
<transmission name=\"eye_r_vert_trans\">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name=\"eyer_link\"/>
    <actuator name=\"eye_r_vert_act\">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
<transmission name=\"eye_l_hori_trans\">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name=\"neck3\"/>
    <actuator name=\"eye_l_hori_act\">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
<transmission name=\"eye_l_vert_trans\">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name=\"eyel_link\"/>
    <actuator name=\"eye_l_vert_act\">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>")
    (format out "</robot>~%")
    (close out)
    )
  )

(defun body-draw-test
  (&key
   (data *robot-data*)
   (ids (cdr (assoc :id data))))
  (if (not (and (boundp '*viewer*) *viewer*)) (make-irtviewer))
  (mapcar
   #'(lambda (a) (send (id-coords :id a) :draw-on :flush nil :width 1000))
   ids)
  (send *viewer* :viewsurface :flush)
  )

(defun cog-draw-test
  (&key
   (data *robot-data*)
   (ids (cdr (assoc :id data))))
  (if (not (and (boundp '*viewer*) *viewer*)) (make-irtviewer))
  (mapcar
   #'(lambda (id)
       (let ((pos (position id (cdr (assoc :id data)))))
	 (send (send
		(copy-object (id-coords :id id))
		:transform
		(make-coords
		 :pos (float-vector
		       (nth pos (cdr (assoc :cx data)))
		       (nth pos (cdr (assoc :cy data)))
		       (nth pos (cdr (assoc :cz data)))))
		:local)
	       :draw-on :flush nil :width 1000)))
   ids)
  (send *viewer* :viewsurface :flush)
  )
