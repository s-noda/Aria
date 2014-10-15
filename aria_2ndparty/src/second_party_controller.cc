#include "common/ssb_common_common.h"
#include "utils/ssb_utils_model.h"
#include "utils/ssb_utils_keyframe-inl.h"
#include <sensor_msgs/Joy.h>

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

#define SENSITIVITY 0.05


namespace vec = ssb_common_vec;
namespace mdl = ssb_utils_model;
namespace key = ssb_utils_keyframe;

class SecondPartyController
{
public:
  SecondPartyController(ros::NodeHandle &nh);
  void Main();
  int getFPS();
private:
  void Callback(const sensor_msgs::Joy::ConstPtr& joy);
  int target_tentacle_;
  bool toggle_wait_;
  vec::VecGripper gripper_accel_;
  vec::VecEye eye_accel_;
  float tentacle_accel_;
  vec::VecGripper gripper_;
  vec::VecEye eye_;
  vec::VecTentacle tentacle_;
  boost::shared_ptr<mdl::GripperModel> gripper_model_;
  boost::shared_ptr<mdl::QuadOEyeModel> eye_model_;
  boost::shared_ptr<mdl::TentacleModel> tentacle_model_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  int _fps_;
  float gripper_speed_;
  float eye_speed_;
  float tentacle_speed_;
};


SecondPartyController::SecondPartyController(ros::NodeHandle &nh) : gripper_(0, 0),
                                                    eye_(0, 0),
                                                    tentacle_(0, 0, 0, 0, 0, 0, 0),
                                                    gripper_accel_(0, 0),
                                                    eye_accel_(0, 0)
{
  nh_ = nh;
  _fps_ = 20;
  gripper_speed_ = 0.1;
  eye_speed_ = 1.0;
  tentacle_speed_ = 2;
  // model init
  gripper_model_
    = boost::shared_ptr<mdl::GripperModel>(new mdl::GripperModel(nh));
  eye_model_
    = boost::shared_ptr<mdl::QuadOEyeModel>(new mdl::QuadOEyeModel(nh));
  tentacle_model_
    = boost::shared_ptr<mdl::TentacleModel>(new mdl::TentacleModel(nh));
  //-------- for joy control --------
  subscriber_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &SecondPartyController::Callback, this);
  toggle_wait_ = true;
  target_tentacle_ = 2;
  tentacle_accel_ = 0;
}

void SecondPartyController::Main()
{
  // Control Gripper.
  gripper_.right += SENSITIVITY*gripper_accel_.right*3;
  if (gripper_.right > 3.0) gripper_.right = 3.0;
  else if (gripper_.right < -0.5) gripper_.right = -0.5;
  gripper_.left -= SENSITIVITY*gripper_accel_.left*3;
  if (gripper_.left > 3.0) gripper_.left = 3.0;
  else if (gripper_.left < -0.5) gripper_.left = -0.5;
  // Control Eye.
  eye_.horizontal += SENSITIVITY*eye_accel_.horizontal;
  if (eye_.horizontal > 1.0) eye_.horizontal = 1.0;
  else if (eye_.horizontal < -1.0) eye_.horizontal = -1.0;
  eye_.vertical += SENSITIVITY*eye_accel_.vertical;
  if (eye_.vertical > 1.0) eye_.vertical = 1.0;
  else if (eye_.vertical < -1.0) eye_.vertical = -1.0;
  // Control Tentacle.
  tentacle_.joint[target_tentacle_] += tentacle_accel_;
  if (tentacle_.joint[target_tentacle_] > 45)
    tentacle_.joint[target_tentacle_] = 45;
  else if (tentacle_.joint[target_tentacle_] < -45)
    tentacle_.joint[target_tentacle_] = -45;
  // Set inputs.
  eye_model_->set_input(vec::VecEye(eye_));
  gripper_model_->set_input(vec::VecGripper(gripper_));
  tentacle_model_->set_input(vec::VecTentacle(tentacle_));
  // Convert input to outputs.
  eye_model_->Input2Output();
  gripper_model_->Input2Output();
  tentacle_model_->Input2Output();
  // Publish outputs.
  eye_model_->send();
  gripper_model_->send();
  tentacle_model_->send();
}

int SecondPartyController::getFPS()
{
  return _fps_;
}

void SecondPartyController::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Set gripper accel.
  if (joy->axes[PS3_AXIS_BUTTON_REAR_LEFT_1] < -0.4) {
    if (fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) < 0.5)
      gripper_accel_.left = 0;
    else
      gripper_accel_.left = gripper_speed_*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
          /fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);
  } else {
    gripper_accel_.left = 0;
  }
  if (joy->axes[PS3_AXIS_BUTTON_REAR_RIGHT_1] < -0.4) {
    if (fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) < 0.5)
      gripper_accel_.right = 0;
    else
      gripper_accel_.right = gripper_speed_*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
          /fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);
  } else {
    gripper_accel_.right = 0;
  }
  // Set eye accel.
  eye_accel_.horizontal = -eye_speed_*joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
  eye_accel_.vertical = eye_speed_*joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
  // Set tentacle target and accel.
  if (joy->buttons[PS3_BUTTON_CROSS_UP]) {
    if ((target_tentacle_ < 4) && toggle_wait_ ) {
      target_tentacle_++;
      toggle_wait_ = false;
    }
  } else if (joy->buttons[PS3_BUTTON_CROSS_DOWN]) {
    if ((target_tentacle_ > 2) && toggle_wait_) {
      target_tentacle_--;
      toggle_wait_ = false;
    }
  } else {
    toggle_wait_ = true;
  }
  if (joy->axes[PS3_AXIS_BUTTON_CROSS_RIGHT] < -0.5) {
    tentacle_accel_ = tentacle_speed_;
  } else if (joy->axes[PS3_AXIS_BUTTON_CROSS_LEFT] < -0.5) {
    tentacle_accel_ = -tentacle_speed_;
  } else {
    tentacle_accel_ = 0;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "second_party_controller");
  ros::NodeHandle nh;
  SecondPartyController controller(nh);
  ros::Rate loop(controller.getFPS());
  while(ros::ok()) {
    controller.Main();
    ros::spinOnce();
    loop.sleep();
  }
};
