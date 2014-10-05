#ifndef SSB_ACTUATORS_KDUINOSERVO_H_
#define SSB_ACTUATORS_KDUINOSERVO_H_

#include "ssb_common_common.h"
#include "kduino/KduinoServo.h"

namespace ssb_actuators_kduinoservo {

template<typename T> class KduinoServo {
 public:
  KduinoServo() {};
  virtual ~KduinoServo() {};
  inline void publish(int id, int value) {
    kduino::KduinoServo msg;
    msg.servo_id = id;
    msg.servo_value = value;
    command_.publish(msg);
  };
  inline void set_motor_id(T motor_id) { motor_id_ = motor_id; };
 protected:
  ros::Publisher command_;
  T motor_id_;
};


class QuadOEyeServo : public KduinoServo<ssb_common_vec::VecQuadOEye> {
 public:
  explicit QuadOEyeServo(ros::NodeHandle &nh) {
    command_ = nh.advertise<kduino::KduinoServo>("/kduino/eyes_servos_control", 100);
  };
  ~QuadOEyeServo() {};
  inline void publish(ssb_common_vec::VecQuadOEye vec) {
    KduinoServo::publish(motor_id_.horizontal_l, vec.horizontal_l);
    KduinoServo::publish(motor_id_.vertical_l, vec.vertical_l);
    KduinoServo::publish(motor_id_.horizontal_r, vec.horizontal_r);
    KduinoServo::publish(motor_id_.vertical_r, vec.vertical_r);
  };
};


class TentacleServo : public KduinoServo<ssb_common_vec::VecTentacle> {
 public:
  explicit TentacleServo(ros::NodeHandle &nh) {
    command_ = nh.advertise<kduino::KduinoServo>("/kduino/ahoge_servos_control", 100);
  };
  ~TentacleServo() {};
  inline void publish(ssb_common_vec::VecTentacle vec) {
    for (int i = 0; i < 7; ++i)
      KduinoServo::publish(motor_id_.joint[i], vec.joint[i]);
  };
};

} // namespace ssb_actuators_kduinoservo

#endif
