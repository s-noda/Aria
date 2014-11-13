#include <iostream>
#include <fstream>
#include "../../aria_2ndparty/src/actuators/virtual_aria.h"
#include "../../aria_2ndparty/src/common/ssb_common_common.h"
#include "../../aria_2ndparty/src/utils/ssb_utils_model.h"
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

class PS3toWeb
{
public:
  PS3toWeb(ros::NodeHandle &nh);
  void Main();
  int getFPS(){ return _fps_; };
private:
  // button action functions
  bool ButtonAction(const sensor_msgs::Joy::ConstPtr& joy, int ps3_key,
                    void (PS3toWeb::*func)(
                        const sensor_msgs::Joy::ConstPtr& joy),
                    void (PS3toWeb::*exception)(
                        const sensor_msgs::Joy::ConstPtr& joy)
                    = &PS3toWeb::ActionNone);
  bool HoldAction(const sensor_msgs::Joy::ConstPtr& joy, int ps3_key,
                  void (PS3toWeb::*func)(
                      const sensor_msgs::Joy::ConstPtr& joy),
                  void (PS3toWeb::*exception)(
                      const sensor_msgs::Joy::ConstPtr& joy)
                  = &PS3toWeb::ActionNone);
  void ActionNone(const sensor_msgs::Joy::ConstPtr& joy);
  void ActionSavePose(const sensor_msgs::Joy::ConstPtr& joy);
  void ActionSwitch2Menu(const sensor_msgs::Joy::ConstPtr& joy);
  void ActionSwitch2Main(const sensor_msgs::Joy::ConstPtr& joy);
 // joy callback functions
  void (PS3toWeb::*joyfunc_)(const sensor_msgs::Joy::ConstPtr& joy);
  void JoyMenuInit(const sensor_msgs::Joy::ConstPtr& joy);
  void JoyMainInit(const sensor_msgs::Joy::ConstPtr& joy);
  void JoyMenu(const sensor_msgs::Joy::ConstPtr& joy);
  void JoyMain(const sensor_msgs::Joy::ConstPtr& joy);
  void Callback(const sensor_msgs::Joy::ConstPtr& joy);
  void Read(const std_msgs::String::ConstPtr& msg);
  void Write(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void SensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  // member variables
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::Subscriber file_subscriber_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber sensor_subscriber_;
  int _fps_;
  bool flags_[19];
  aria::currenter sensor_;
  vec::VecGripper gripper_;
  vec::VecEye eye_;
};


PS3toWeb::PS3toWeb(ros::NodeHandle &nh)
{
  nh_ = nh;
  _fps_ = 30;
  for (int i=0; i<19; ++i)
    flags_[i] = false;
  publisher_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "/concatenate_request/flow", 1000);
  subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(
      "/concatenate_request/stock", 1000, &PS3toWeb::Write, this);
  file_subscriber_ = nh_.subscribe<std_msgs::String>(
      "/aria_web_ui/file_request", 1000, &PS3toWeb::Read, this);
  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>(
      "/joy", 1000, &PS3toWeb::Callback, this);
  sensor_subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(
      "/currentor_socket/sensor_array/position", 1000, &PS3toWeb::SensorCallback, this);
  joyfunc_ = &PS3toWeb::JoyMainInit;
}

void PS3toWeb::Main()
{
  // This is written here because Callback is only called
  // when there are key press differences.
}

void PS3toWeb::
Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  (this->*joyfunc_)(joy);
}

void PS3toWeb::
SensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  sensor_.position = msg->data;
}

void PS3toWeb::
Read(const std_msgs::String::ConstPtr& msg)
{
  std::string line;
  std::ifstream myfile(msg->data.c_str());
  std::vector<std::vector<float> > pose;
  if (myfile.is_open()) {
    getline(myfile, line);
    std::stringstream ss0(line);
    float number_of_pose, joints;
    ss0 >> number_of_pose >> joints;
    pose.resize(number_of_pose);
    int i = 0;
    while (getline(myfile, line)) {
      std::stringstream ss(line);
      pose[i].resize(joints);
      for (int j=0; j<joints; ++j) {
	ss >> pose[i].at(j);
      }
      i++;
    }
    for (int i=0; i<pose.size(); ++i) {
      std_msgs::Float32MultiArray msg;
      msg.data.resize(pose[i].size());
      msg.data = pose[i];
      publisher_.publish(msg);
    }
  }
}

void PS3toWeb::
Write(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  std::ofstream myfile;
  std::vector<float> data;
  data.resize(msg->data[1]);
  float joints = msg->data[2];
  myfile.open(ofToString(static_cast<int>(msg->data[0])).c_str());
  myfile << msg->data[1] << " " << msg->data[2] << "\n";
  for (int i=0; i<data.size(); ++i) {
    for (int j=0; j<joints; ++j)
      myfile << msg->data[3+i*joints+j] << " ";
    myfile << "\n";
  }
  myfile.close();
  ROS_WARN("saved data %s",ofToString(static_cast<int>(msg->data[0])).c_str());
}


bool PS3toWeb::
ButtonAction(const sensor_msgs::Joy::ConstPtr& joy,
             int ps3_key,
             void (PS3toWeb::*func)(
                 const sensor_msgs::Joy::ConstPtr& joy),
             void (PS3toWeb::*exception)(
                 const sensor_msgs::Joy::ConstPtr& joy))
{
  if ((joy->axes[ps3_key] < -0.5)) {
    if (flags_[ps3_key])
      return false;
    flags_[ps3_key] = true;
    (this->*func)(joy);
    return true;
  } else {
    flags_[ps3_key] = false;
    (this->*exception)(joy);
    return false;
  }
}

bool PS3toWeb::
HoldAction(const sensor_msgs::Joy::ConstPtr& joy,
           int ps3_key,
           void (PS3toWeb::*func)(
               const sensor_msgs::Joy::ConstPtr& joy),
           void (PS3toWeb::*exception)(
               const sensor_msgs::Joy::ConstPtr& joy))
{
  if ((joy->axes[ps3_key] < -0.4)) {
    flags_[ps3_key] = true;
    (this->*func)(joy);
    return true;
  } else {
    flags_[ps3_key] = false;
    (this->*exception)(joy);
    return false;
  }
}

void PS3toWeb::
ActionNone(const sensor_msgs::Joy::ConstPtr& joy)
{
}

void PS3toWeb::
ActionSavePose(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::Float32MultiArray msg;
  int data_size = sensor_.position.size();
  msg.data.resize(data_size);
  msg.data = sensor_.position;
  msg.data.resize(data_size+5);
  msg.data.at(data_size) = gripper_.left;
  msg.data.at(data_size+1) = gripper_.right;
  msg.data.at(data_size+2) = eye_.horizontal;
  msg.data.at(data_size+3) = eye_.vertical;
  msg.data.at(data_size+4) = 1.0; // extra element for time
  publisher_.publish(msg);
  ROS_WARN("saved pose");
}

void PS3toWeb::
ActionSwitch2Menu(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyfunc_ = &PS3toWeb::JoyMenuInit;
}

void PS3toWeb::
ActionSwitch2Main(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyfunc_ = &PS3toWeb::JoyMainInit;
}


void PS3toWeb::
JoyMenuInit(const sensor_msgs::Joy::ConstPtr& joy) {
  ROS_WARN("switched to menu");
  ROS_INFO("%s\n %s\n",
           "Menu:",
           " triangle: return");
  joyfunc_ = &PS3toWeb::JoyMenu;
}

void PS3toWeb::
JoyMenu(const sensor_msgs::Joy::ConstPtr& joy) {
  ButtonAction(joy, PS3_AXIS_BUTTON_ACTION_TRIANGLE,
               &PS3toWeb::ActionSwitch2Main);
}

void PS3toWeb::
JoyMainInit(const sensor_msgs::Joy::ConstPtr& joy) {
  ROS_WARN("switched to head and gripper");
  joyfunc_ = &PS3toWeb::JoyMain;
}

void PS3toWeb::
JoyMain(const sensor_msgs::Joy::ConstPtr& joy) {
  ButtonAction(joy, PS3_AXIS_BUTTON_ACTION_TRIANGLE,
               &PS3toWeb::ActionSwitch2Menu);
  ButtonAction(joy, PS3_AXIS_BUTTON_ACTION_CIRCLE,
               &PS3toWeb::ActionSavePose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "second_party_controller");
  ros::NodeHandle nh;
  PS3toWeb controller(nh);
  ros::Rate loop(controller.getFPS());
  while(ros::ok()) {
    controller.Main();
    ros::spinOnce();
    loop.sleep();
  }
};
