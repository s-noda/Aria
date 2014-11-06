#include "common/ssb_common_common.h"
#include "utils/ssb_utils_model.h"
#include "utils/ssb_utils_keyframe-inl.h"
#include <std_msgs/Float32MultiArray.h>

namespace vec = ssb_common_vec;
namespace mdl = ssb_utils_model;
namespace key = ssb_utils_keyframe;

class SecondPartyCore
{
public:
  SecondPartyCore(ros::NodeHandle &nh);
  void Main();
  int getFPS();
private:
  void GripperInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
  void EyeInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
  void TentacleInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req);
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
  ros::Subscriber gripper_subscriber_;
  ros::Subscriber eye_subscriber_;
  ros::Subscriber tentacle_subscriber_;
  int _fps_;
  // for virtual models
  boost::shared_ptr<mdl::VirtualGripperModel> virtual_gripper_model_;
  boost::shared_ptr<mdl::VirtualQuadOEyeModel> virtual_eye_model_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecGripper, mdl::VirtualGripperModel> > virtual_gripper_player_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecEye, mdl::VirtualQuadOEyeModel> > virtual_eye_player_;
};


SecondPartyCore::SecondPartyCore(ros::NodeHandle &nh)
{
  nh_ = nh;
  _fps_ = 100;
  // model init
  gripper_model_
    = boost::shared_ptr<mdl::GripperModel>(new mdl::GripperModel(nh));
  eye_model_
    = boost::shared_ptr<mdl::QuadOEyeModel>(new mdl::QuadOEyeModel(nh));
  tentacle_model_
    = boost::shared_ptr<mdl::TentacleModel>(new mdl::TentacleModel(nh));
  //-------- for interpolation --------
  gripper_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/gripper", 10, &SecondPartyCore::GripperInterpolation, this);
  eye_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/eye", 10, &SecondPartyCore::EyeInterpolation, this);
  tentacle_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>
      ("/2ndparty/request/tentacle", 10, &SecondPartyCore::TentacleInterpolation, this);
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
  // virtual init
  virtual_gripper_model_
    = boost::shared_ptr<mdl::VirtualGripperModel>(new mdl::VirtualGripperModel(nh));
  virtual_eye_model_
    = boost::shared_ptr<mdl::VirtualQuadOEyeModel>(new mdl::VirtualQuadOEyeModel(nh));
  virtual_gripper_player_ = boost::shared_ptr<key::KeyFramePlayer
                                      <vec::VecGripper, mdl::VirtualGripperModel> >
      (new key::KeyFramePlayer<vec::VecGripper, mdl::VirtualGripperModel>
       ("virtual_gripper", gripper_key_, *virtual_gripper_model_, _fps_));
  virtual_eye_player_ = boost::shared_ptr<key::KeyFramePlayer
                                  <vec::VecEye, mdl::VirtualQuadOEyeModel> >
      (new key::KeyFramePlayer<vec::VecEye, mdl::VirtualQuadOEyeModel>
       ("virtual_eye", eye_key_, *virtual_eye_model_, _fps_));
}

void SecondPartyCore::Main()
{
  gripper_player_->OnPlay();
  eye_player_->OnPlay();
  tentacle_player_->OnPlay();
  // play virtual model
  virtual_gripper_player_->OnPlay();
  virtual_eye_player_->OnPlay();
}

int SecondPartyCore::getFPS()
{
  return _fps_;
}

void SecondPartyCore::GripperInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req)
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
  // Start virtual model motion.
  virtual_gripper_player_->Setup();
  virtual_gripper_player_->StartPlay();
}

void SecondPartyCore::EyeInterpolation(const std_msgs::Float32MultiArray::ConstPtr& req)
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
  // Start virtual model motion.
  virtual_eye_player_->Setup();
  virtual_eye_player_->StartPlay();
}

void SecondPartyCore::TentacleInterpolation(const std_msgs::Float32MultiArray::ConstPtr &req)
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
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "second_party_core");
  ros::NodeHandle nh;
  SecondPartyCore controller(nh);
  ros::Rate loop(controller.getFPS());
  while(ros::ok()) {
    controller.Main();
    ros::spinOnce();
    loop.sleep();
  }
};
