#include "common/ssb_common_common.h"
#include "utils/ssb_utils_model.h"
#include "utils/ssb_utils_keyframe-inl.h"
#include <std_msgs/Float32MultiArray.h>
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

class SencondPartyController {
public:
  SencondPartyController(ros::NodeHandle &nh);
  void Main();
  void Sub();
  int getFPS();
  bool getFlag();
private:
  void Callback(const sensor_msgs::Joy::ConstPtr& joy);
  void GripperInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
  void EyeInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
  void TentacleInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
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
  std::vector<key::KeyFrame<vec::VecGripper> > gripper_key_;
  std::vector<key::KeyFrame<vec::VecEye> > eye_key_;
  std::vector<key::KeyFrame<vec::VecTentacle> > tentacle_key_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecGripper, mdl::GripperModel> > gripper_player_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecEye, mdl::QuadOEyeModel> > eye_player_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecTentacle, mdl::TentacleModel> > tentacle_player_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::Subscriber gripper_subscriber_;
  ros::Subscriber eye_subscriber_;
  ros::Subscriber tentacle_subscriber_;
  int _fps_;
  bool joy_flag_;
};


SencondPartyController::SencondPartyController(ros::NodeHandle &nh) : gripper_(0, 0),
                                                    eye_(0, 0),
                                                    tentacle_(0, 0, 0, 0, 0, 0, 0),
                                                    gripper_accel_(0, 0),
                                                    eye_accel_(0, 0)
{
  nh_ = nh;
  _fps_ = 20;
  // model init
  gripper_model_
    = boost::shared_ptr<mdl::GripperModel>(new mdl::GripperModel(nh));
  eye_model_
    = boost::shared_ptr<mdl::QuadOEyeModel>(new mdl::QuadOEyeModel(nh));
  tentacle_model_
    = boost::shared_ptr<mdl::TentacleModel>(new mdl::TentacleModel(nh));
  //-------- for joy control --------
  subscriber_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &SencondPartyController::Callback, this);
  joy_flag_ = true;
  toggle_wait_ = true;
  target_tentacle_ = 2;
  tentacle_accel_ = 0;
  //-------- for interpolation --------
  gripper_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/gripper", 10, &SencondPartyController::GripperInterpolation, this);
  eye_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/eye", 10, &SencondPartyController::EyeInterpolation, this);
  tentacle_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/tentacle", 10, &SencondPartyController::TentacleInterpolation, this);
  // player init
  gripper_player_ = boost::shared_ptr<key::KeyFramePlayer
                                      <vec::VecGripper, mdl::GripperModel> >
      (new key::KeyFramePlayer<vec::VecGripper, mdl::GripperModel>
       ("gripper", gripper_key_, *gripper_model_, _fps_));
  eye_player_ = boost::shared_ptr<key::KeyFramePlayer
                                  <vec::VecEye, mdl::QuadOEyeModel> >
      (new key::KeyFramePlayer<vec::VecEye, mdl::QuadOEyeModel>
       ("eye", eye_key_, *eye_model_, _fps_));
  tentacle_player_ = boost::shared_ptr<key::KeyFramePlayer
                                       <vec::VecTentacle, mdl::TentacleModel> >
      (new key::KeyFramePlayer<vec::VecTentacle, mdl::TentacleModel>
       ("tentacle", tentacle_key_, *tentacle_model_, _fps_));
  // keyframe init
  std::vector<ofVec2f> tmp;
  key::KeyFrame<vec::VecGripper>
      gripper_frame(vec::VecGripper(0, 0), vec::VecTime(0, 0), vec::VecInterpolation(1, tmp));
  key::KeyFrame<vec::VecGripper>
      gripper_frame1(vec::VecGripper(0, 0), vec::VecTime(1, 0), vec::VecInterpolation(1, tmp));
  gripper_key_.push_back(gripper_frame);
  gripper_key_.push_back(gripper_frame1);
  key::KeyFrame<vec::VecEye>
      eye_frame(vec::VecEye(0, 0), vec::VecTime(0, 0), vec::VecInterpolation(1, tmp));
  key::KeyFrame<vec::VecEye>
      eye_frame1(vec::VecEye(0, 0), vec::VecTime(1, 0), vec::VecInterpolation(1, tmp));
  eye_key_.push_back(eye_frame);
  eye_key_.push_back(eye_frame1);
  key::KeyFrame<vec::VecTentacle>
      tentacle_frame(vec::VecTentacle(0, 0, 0, 0, 0, 0, 0), vec::VecTime(0, 0),
                     vec::VecInterpolation(1, tmp));
  key::KeyFrame<vec::VecTentacle>
      tentacle_frame1(vec::VecTentacle(0, 0, 0, 0, 0, 0, 0), vec::VecTime(1, 0),
                      vec::VecInterpolation(1, tmp));
  tentacle_key_.push_back(tentacle_frame);
  tentacle_key_.push_back(tentacle_frame1);
}

void SencondPartyController::Main()
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

void SencondPartyController::Sub()
{
  gripper_player_->OnPlay();
  eye_player_->OnPlay();
  tentacle_player_->OnPlay();
}

int SencondPartyController::getFPS()
{
  return _fps_;
}

bool SencondPartyController::getFlag()
{
  return joy_flag_;
}

void SencondPartyController::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (joy->axes[PS3_AXIS_BUTTON_ACTION_CROSS] < -0.5) {
    ROS_INFO("controller on");
    joy_flag_ = true;
  }
  // Set gripper accel.
  if (joy->axes[PS3_AXIS_BUTTON_REAR_LEFT_1] < -0.4) {
    if (fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) < 0.5)
      gripper_accel_.left = 0;
    else
      gripper_accel_.left = 0.1*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
          /fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);
  } else {
    gripper_accel_.left = 0;
  }
  if (joy->axes[PS3_AXIS_BUTTON_REAR_RIGHT_1] < -0.4) {
    if (fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) < 0.5)
      gripper_accel_.right = 0;
    else
      gripper_accel_.right = 0.1*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]
          /fabsf(joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]);
  } else {
    gripper_accel_.right = 0;
  }
  // Set eye accel.
  eye_accel_.horizontal = -joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
  eye_accel_.vertical = joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
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
    tentacle_accel_ = 2;
  } else if (joy->axes[PS3_AXIS_BUTTON_CROSS_LEFT] < -0.5) {
    tentacle_accel_ = -2;
  } else {
    tentacle_accel_ = 0;
  }
}


void SencondPartyController::GripperInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req)
{
  std::vector<ofVec2f> tmp;
  // Get previous pose.
  if (gripper_key_.size() > 1) {
    key::KeyFrame<vec::VecGripper> tmp_frame(gripper_key_.at(1));
    tmp_frame.set_time_data(vec::VecTime(0, 0));
    gripper_key_.clear();
    gripper_key_.push_back(tmp_frame);
  }
  // Set next pose.
  key::KeyFrame<vec::VecGripper>
      gripper_frame(vec::VecGripper(req->data[0], req->data[1]),
                 vec::VecTime(req->data[2], 0),
                 vec::VecInterpolation(1, tmp));
  gripper_key_.push_back(gripper_frame);
  // Start motion.
  gripper_player_->Setup();
  gripper_player_->StartPlay();
  joy_flag_ = false;
}

void SencondPartyController::EyeInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req)
{
  std::vector<ofVec2f> tmp;
  // Get previous pose.
  if (eye_key_.size() > 1) {
    key::KeyFrame<vec::VecEye> tmp_frame(eye_key_.at(1));
    tmp_frame.set_time_data(vec::VecTime(0, 0));
    eye_key_.clear();
    eye_key_.push_back(tmp_frame);
  }
  // Set next pose.
  key::KeyFrame<vec::VecEye>
      eye_frame(vec::VecEye(req->data[0], req->data[1]),
                vec::VecTime(req->data[2], 0),
                vec::VecInterpolation(1, tmp));
  eye_key_.push_back(eye_frame);
  // Start motion.
  eye_player_->Setup();
  eye_player_->StartPlay();
  joy_flag_ = false;
}

void SencondPartyController::TentacleInterpolation(const std_msgs::Float32MultiArray::ConstPtr &req)
{
  std::vector<ofVec2f> tmp;
  // Get previous pose.
  if (tentacle_key_.size() > 1) {
    key::KeyFrame<vec::VecTentacle> tmp_frame(tentacle_key_.at(1));
    tmp_frame.set_time_data(vec::VecTime(0, 0));
    tentacle_key_.clear();
    tentacle_key_.push_back(tmp_frame);
  }
  // Set next pose.
  key::KeyFrame<vec::VecTentacle>
      tentacle_frame(vec::VecTentacle(req->data[0], req->data[1],
                                      req->data[2], req->data[3],
                                      req->data[4], 0, 0),
                     vec::VecTime(req->data[5], 0),
                     vec::VecInterpolation(1, tmp));
  tentacle_key_.push_back(tentacle_frame);
  // Start motion.
  tentacle_player_->Setup();
  tentacle_player_->StartPlay();
  joy_flag_ = false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "second_party_controller");
  ros::NodeHandle nh;
  SencondPartyController controller(nh);
  ros::Rate loop(controller.getFPS());
  while(ros::ok()) {
    if (controller.getFlag())
      controller.Main();
    else
      controller.Sub();
    ros::spinOnce();
    loop.sleep();
  }
};
