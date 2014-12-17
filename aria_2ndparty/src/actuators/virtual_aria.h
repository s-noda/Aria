#ifndef ARIA_UTILS_VIRTUAL_ARIA_H_
#define ARIA_UTILS_VIRTUAL_ARIA_H_

#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <unistd.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "ssb_common_common.h"
#include "ssb_common_vec.h"

namespace aria {

struct currenter {
  std::vector<float> mode;
  std::vector<float> torque;
  std::vector<float> position;
};

class VirtualAria {
 public:
  explicit VirtualAria(ros::NodeHandle &nh) {
    nh_ = nh;
    mode_publisher_ =
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/virtual_array/mode", 1000);
    torque_publisher_ =
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/virtual_array/torque", 1000);
    position_publisher_ =
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/virtual_array/position", 1000);
    mode_subscriber_ =
        nh_.subscribe("/currentor_socket/request/mode_vector", 1000,
		      &VirtualAria::ModeRequestCallback, this);
    c_.mode.resize(30);
    c_.torque.resize(30);
    c_.position.resize(30);
    for (int i = 0; i < c_.mode.size(); ++i)
      c_.mode[i] = 2;
  };
  inline void publish(ssb_common_vec::VecBody vec) {
    std_msgs::Float32MultiArray msg_m;
    msg_m.data.resize(c_.mode.size());
    msg_m.data = c_.mode;
    mode_publisher_.publish(msg_m);
    if (vec.joints.size() != c_.mode.size())
      return;
    for (int i = 0; i < vec.joints.size(); ++i) {
      if (c_.mode[i] == 1)
	c_.torque[i] = vec.joints[i];
      else if (c_.mode[i] == 2)
	c_.position[i] = vec.joints[i];
    }
    std_msgs::Float32MultiArray msg_t;
    msg_t.data.resize(vec.joints.size());
    msg_t.data = c_.torque;
    torque_publisher_.publish(msg_t);
    std_msgs::Float32MultiArray msg_p;
    msg_p.data.resize(vec.joints.size());
    msg_p.data = c_.position;
    position_publisher_.publish(msg_p);
  };
 protected:
  void ModeRequestCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
    c_.mode = data->data;
  };
  currenter c_;
  ros::NodeHandle nh_;
  ros::Publisher mode_publisher_;
  ros::Publisher torque_publisher_;
  ros::Publisher position_publisher_;
  ros::Subscriber mode_subscriber_;
};

class VirtualForce {
 public:
  explicit VirtualForce(ros::NodeHandle &nh) {
    angular_publisher_ = nh.advertise<std_msgs::Float32MultiArray>("/root_force/virtual_array/angular", 100);
    spacial_publisher_ = nh.advertise<std_msgs::Float32MultiArray>("/root_force/virtual_array/spacial", 100);
  }
  ~VirtualForce() {};
  inline void publish(ssb_common_vec::VecForce vec) {
    std_msgs::Float32MultiArray angular, spacial;
    angular.data.resize(3);
    angular.data.at(0) = vec.ax;
    angular.data.at(1) = vec.ay;
    angular.data.at(2) = vec.az;
    spacial.data.resize(3);
    spacial.data.at(0) = vec.x;
    spacial.data.at(1) = vec.y;
    spacial.data.at(2) = vec.z;
    angular_publisher_.publish(angular);
    spacial_publisher_.publish(spacial);
  };
 private:
  ros::Publisher angular_publisher_;
  ros::Publisher spacial_publisher_;
};

}

#endif
