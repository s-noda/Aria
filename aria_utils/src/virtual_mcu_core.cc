#include "../../aria_2ndparty/src/actuators/virtual_aria.h"
#include "../../aria_2ndparty/src/common/ssb_common_common.h"
#include "../../aria_2ndparty/src/utils/ssb_utils_model.h"
#include "../../aria_2ndparty/src/utils/ssb_utils_keyframe-inl.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

namespace vec = ssb_common_vec;
namespace mdl = ssb_utils_model;
namespace key = ssb_utils_keyframe;

class VirtualMCUCore {
public:
  VirtualMCUCore(ros::NodeHandle &nh);
  void Main();
  int getFPS();
private:
  void JsonRequestCallback(const std_msgs::String& msg);
  void TorqueRequestCallback(const std_msgs::Float32MultiArray::ConstPtr& data);
  void PositionRequestCallback(const std_msgs::Float32MultiArray::ConstPtr& data);
  void TorqueSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& data);
  void PositionSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& data);
  void setGoal(std::vector<float> goal);
  boost::shared_ptr<mdl::VirtualAriaModel> aria_model_;
  std::vector<key::KeyFrame<vec::VecBody> > aria_key_;
  boost::shared_ptr<key::KeyFramePlayer
		    <vec::VecBody, mdl::VirtualAriaModel> > aria_player_;
  ros::NodeHandle nh_;
  ros::Subscriber json_subscriber_;
  ros::Subscriber torque_subscriber_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber tsensor_subscriber_;
  ros::Subscriber psensor_subscriber_;
  int _fps_;
  float set_time_;
  vec::VecInterpolation set_interpolation_;
  aria::currenter sensor_;
};

VirtualMCUCore::VirtualMCUCore(ros::NodeHandle &nh) {
  nh_ = nh;
  _fps_ = 100;
  set_time_ = 0;
  sensor_.mode.resize(30);
  sensor_.torque.resize(30);
  sensor_.position.resize(30);
  aria_model_
    = boost::shared_ptr<mdl::VirtualAriaModel>(new mdl::VirtualAriaModel(nh));
  torque_subscriber_ =
    nh_.subscribe("/currentor_socket/request/torque_vector", 1000,
		  &VirtualMCUCore::TorqueRequestCallback, this);
  position_subscriber_ =
    nh_.subscribe("/currentor_socket/request/position_vector", 1000,
		  &VirtualMCUCore::PositionRequestCallback, this);
  json_subscriber_ = 
    nh_.subscribe("/ros2http/socket_listener/json_string", 1000,
		  &VirtualMCUCore::JsonRequestCallback, this);
  tsensor_subscriber_ =
    nh_.subscribe("/currentor_socket/virtual_array/torque", 1000,
		  &VirtualMCUCore::TorqueSensorCallback, this);
  psensor_subscriber_ =
    nh_.subscribe("/currentor_socket/virtual_array/position", 1000,
		  &VirtualMCUCore::PositionSensorCallback, this);
  aria_player_ = boost::shared_ptr<key::KeyFramePlayer
				   <vec::VecBody, mdl::VirtualAriaModel> >
    (new key::KeyFramePlayer<vec::VecBody, mdl::VirtualAriaModel>
     ("virtual_aria", aria_key_, *aria_model_, _fps_));
  // keyframe init
  std::vector<ofVec2f> tmp;
  key::KeyFrame<vec::VecBody>
    aria_frame(vec::VecBody(30), vec::VecTime(0,0), vec::VecInterpolation(1, tmp));
  key::KeyFrame<vec::VecBody>
    aria_frame1(vec::VecBody(30), vec::VecTime(0,0), vec::VecInterpolation(1, tmp));
  aria_key_.push_back(aria_frame);
  aria_key_.push_back(aria_frame1);
  set_interpolation_ = vec::VecInterpolation(1, tmp);
}

void VirtualMCUCore::Main() {
  aria_player_->OnPlay();
}

int VirtualMCUCore::getFPS() {
  return _fps_;
}

void VirtualMCUCore::setGoal(std::vector<float> goal) {
  std::vector<ofVec2f> tmp;
  // Get previous pose.
  if (aria_key_.size() > 1) {
    key::KeyFrame<vec::VecBody> tmp_frame(aria_key_.at(1));
    tmp_frame.set_time_data(vec::VecTime(0, 0));
    tmp_frame.set_interpolation_data(set_interpolation_);
    aria_key_.clear();
    aria_key_.push_back(tmp_frame);
  }
  // Set next pose.
  key::KeyFrame<vec::VecBody>
    aria_frame(vec::VecBody(goal),
	       vec::VecTime(set_time_, 0),
	       vec::VecInterpolation(1, tmp));
  aria_key_.push_back(aria_frame);
  // Start motion.
  aria_player_->Setup();
  aria_player_->StartPlay();
}

void VirtualMCUCore::JsonRequestCallback(const std_msgs::String& data) {
  std::stringstream ss;
  ss << data.data;
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(ss, pt);
  std::vector<float> params;
  boost::optional<std::string> method = pt.get_optional<std::string>("method");
  boost::optional<std::string> param_str = pt.get_optional<std::string>("params");
  char *token = strtok(const_cast<char*>(param_str.get().c_str()), "[,]");
  while (token != NULL) {
    params.push_back(atof(token));
    token = strtok(NULL, "[,]");
  }
  if (method.get() == "setTime") {
    if (params.size() < 1)
      return;
    set_time_ = params.at(0);
  }
  if (method.get() == "setPosition") {
    if (params.size() < 2)
      return;
    std::vector<float> dat;
    dat.resize(sensor_.position.size());
    dat = sensor_.position;
    dat[params.at(0)] = params.at(1);
    setGoal(dat);
  }
  if (method.get() == "setTorque") {
    if (params.size() < 2)
      return;
    std::vector<float> dat;
    dat.resize(sensor_.torque.size());
    dat = sensor_.torque;
    dat[params.at(0)] = params.at(1);
    setGoal(dat);
  }
  if (method.get() == "setInterpolation") {
    if (params.size() < 1)
      return;
    set_interpolation_.type = params.at(0);
    switch(set_interpolation_.type) {
    case 2 :
      //if (params.size() < 2)
      //	return;
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(3);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      set_interpolation_.p[1] = ofVec2f(0.0, 1.0);//ofVec2f(0.0, params.at(1));
      set_interpolation_.p[2] = ofVec2f(1.0, 1.0);
      break;
    case 3 :
      //if (params.size() < 3)
      //	return;
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(4);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      set_interpolation_.p[1] = ofVec2f(0.5, 0.8);//ofVec2f(params.at(1), params.at(2));
      set_interpolation_.p[2] = ofVec2f(0.5/0.8, 1.0);//ofVec2f(params.at(1)/params.at(2), 1.0);
      set_interpolation_.p[3] = ofVec2f(1.0, 1.0);
    case 4 :
      //if (params.size() < 3)
      //	return;
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(4);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      set_interpolation_.p[1] = ofVec2f(0.3/0.8, 0.0);//ofVec2f((params.at(1)-params.at(2))/(1-params.at(2)), 0.0);
      set_interpolation_.p[2] = ofVec2f(0.5, 0.2);//ofVec2f(params.at(1), params.at(2));
      set_interpolation_.p[3] = ofVec2f(1.0, 1.0);
      break;
    case 5 :
      //if (params.size() < 4)
      //	return;
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(6);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      set_interpolation_.p[1] = ofVec2f(0.01/0.6, 0.0);//ofVec2f((params.at(1)*params.at(4)-params.at(3)*params.at(2))/(params.at(4)-params.at(2)),0.0);
      set_interpolation_.p[2] = ofVec2f(0.3, 0.2);//ofVec2f(params.at(1), params.at(2));
      set_interpolation_.p[3] = ofVec2f(0.7, 0.8);//ofVec2f(params.at(3), params.at(4));
      set_interpolation_.p[4] = ofVec2f(0.05/0.6, 1.0);//ofVec2f((params.at(3)*(1-params.at(2))-params.at(1)*(1-params.at(4)))/(params.at(4)-params.at(2)),1.0);
      set_interpolation_.p[5] = ofVec2f(1.0, 1.0);
      break;
    case 6 :
      //if (params.size() < 3)
      //	return;
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(4);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      set_interpolation_.p[1] = ofVec2f(0.0, 0.5);//ofVec2f(0.0, params.at(1));
      set_interpolation_.p[2] = ofVec2f(1.0, 0.5);//ofVec2f(1.0, params.at(2));
      set_interpolation_.p[3] = ofVec2f(1.0, 1.0);
      break;
    default:
      set_interpolation_.p.clear();
      set_interpolation_.p.resize(1);
      set_interpolation_.p[0] = ofVec2f(0.0, 0.0);
      break;
    }
  }
}

void VirtualMCUCore::TorqueRequestCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
  set_time_ = data->data.at(data->data.size()-1);
  setGoal(data->data);
}

void VirtualMCUCore::PositionRequestCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
  set_time_ = data->data.at(data->data.size()-1);
  setGoal(data->data);
}

void VirtualMCUCore::TorqueSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
  sensor_.torque = data->data;
}

void VirtualMCUCore::PositionSensorCallback(const std_msgs::Float32MultiArray::ConstPtr &data) {
  sensor_.position = data->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "virtual_mcu_core");
  ros::NodeHandle nh;
  VirtualMCUCore core(nh);
  ros::Rate loop(core.getFPS());
  while(ros::ok()) {
    core.Main();
    ros::spinOnce();
    loop.sleep();
  }
};
