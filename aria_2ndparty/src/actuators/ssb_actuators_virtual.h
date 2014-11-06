#ifndef SSB_ACTUATORS_VIRTUAL_H_
#define SSB_ACTUATORS_VIRTUAL_H_

#include "ssb_common_common.h"
#include "std_msgs/Float32MultiArray.h"

namespace ssb_actuators_virtual {

class VirtualObject {
 public:
  VirtualObject() {};
  virtual ~VirtualObject() {};
  inline void publish(std::vector<float> value) {
    std_msgs::Float32MultiArray msg;
    msg.data = value;
    command_.publish(msg);
  };
 protected:
  ros::Publisher command_;
};

class VirtualQuadOEyeObject : public VirtualObject {
 public:
  explicit VirtualQuadOEyeObject(ros::NodeHandle &nh) {
    command_ = nh.advertise<std_msgs::Float32MultiArray>("/2ndparty/virtual_array/eye", 100);
  };
  ~VirtualQuadOEyeObject() {};
  inline void publish(ssb_common_vec::VecQuadOEye vec) {
    std::vector<float> value;
    value.resize(4);
    value[0] = vec.horizontal_r;
    value[1] = vec.vertical_r;
    value[2] = vec.horizontal_l;
    value[3] = vec.vertical_l;
    VirtualObject::publish(value);
  };
};

class VirtualGripperObject : public VirtualObject {
 public:
  explicit VirtualGripperObject(ros::NodeHandle &nh) {
    command_ = nh.advertise<std_msgs::Float32MultiArray>("/2ndparty/virtual_array/gripper", 100);
  };
  ~VirtualGripperObject() {};
  inline void publish(ssb_common_vec::VecGripper vec) {
    std::vector<float> value;
    value.resize(2);
    value[0] = vec.right;
    value[1] = vec.left;
    VirtualObject::publish(value);
  };
};

}

#endif
