#ifndef SSB_ACTUATORS_DYNAMIXEL_H_
#define SSB_ACTUATORS_DYNAMIXEL_H_

#include "ssb_common_common.h"
#include "dynamixel_msgs/MotorStateList.h"
#include "std_msgs/Float64.h"

namespace ssb_actuators_dynamixel {

class Dynamixel : public ssb_common_subscriber::Subscriber<dynamixel_msgs::MotorStateList> {
 public:
  Dynamixel() {};
  virtual ~Dynamixel() {};
  inline void publish(ros::Publisher pub, float value) {
    std_msgs::Float64 msg;
    msg.data = value;
    pub.publish(msg);
  };
};

class GripperDynamixel : public Dynamixel {
 public:
  explicit GripperDynamixel(ros::NodeHandle &nh) {
    right_gripper = nh.advertise<std_msgs::Float64>("/controller1/command", 100);
    left_gripper = nh.advertise<std_msgs::Float64>("/controller2/command", 100);
  };
  inline void publish(ssb_common_vec::VecGripper vec) {
    Dynamixel::publish(right_gripper, vec.right);
    Dynamixel::publish(left_gripper, vec.left);
  };
 private:
  ros::Publisher right_gripper, left_gripper;
};

} // namespace ssb_actuators_dynamixel

#endif
