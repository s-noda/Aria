#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")

(ros::roseus "eus_keyboard")
(ros::advertise "/euslisp/keyboard" std_msgs::String)

(ros::rate 100)

(while (ros::ok)
  (setq key (read-line))
  (setq msg (instance std_msgs::string :init))
  (send msg :data key)
  (ros::publish "/euslisp/keyboard" msg)
  (ros::sleep))