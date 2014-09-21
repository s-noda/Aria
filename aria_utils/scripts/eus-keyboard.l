#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "std_msgs")

(ros::roseus "eus_commandline")
(ros::advertise "/aria/commandline" std_msgs::String)

(ros::rate 100)

(while (ros::ok)
  (setq key (read-line))
  (setq msg (instance std_msgs::string :init))
  (send msg :data key)
  (ros::publish "/aria/commandline" msg)
  (ros::sleep))