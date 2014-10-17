#ifndef ARIA_UTILS_ARIA_H_
#define ARIA_UTILS_ARIA_H_

#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <unistd.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

struct currenter {
  std::vector<float> mode;
  std::vector<float> torque;
  std::vector<float> position;
  std::vector<float> debug;
};

class Aria {
 public:
  explicit Aria(ros::NodeHandle &nh) {
    nh_ = nh;
    json_publisher_ =
        nh_.advertise<std_msgs::String>(
            "/ros2http/socket_listener/json_string", 1000);
    modes_publisher_ =
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/request/mode_vector", 1000);
    torques_publisher_ = 
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/request/torque_vector", 1000);
    positions_publisher_ =
        nh_.advertise<std_msgs::Float32MultiArray>(
            "/currentor_socket/request/position_vector", 1000);
    torque_subscriber_ =
        nh_.subscribe("/currentor_socket/sensor_array/torque", 1000,
                      &Aria::SensorTorqueCallback, this);
    position_subscriber_ =
        nh_.subscribe("/currentor_socket/sensor_array/position", 1000,
                      &Aria::SensorPositionCallback, this);
    debug_subscriber_ =
        nh_.subscribe("/currentor_socket/sensor_array/debug", 1000,
                      &Aria::SensorDebugCallback, this);
  };
  inline void setMode(int joint, int mode) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "{\"method\":\"setControlMode\",\"params\":\"[" << joint
       << "," << mode << "],\"id\":\"1\"}";
    msg.data = ss.str();
    json_publisher_.publish(msg);
  };
  inline void setModes() {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(goal_.mode.size());
    for (int i=0; i<goal_.mode.size(); ++i)
      msg.data[i] = goal_.mode.at(i);
    modes_publisher_.publish(msg);
  };
  inline void setTorques() {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(goal_.torque.size());
    for (int i=0; i<goal_.torque.size(); ++i)
      msg.data[i] = goal_.torque.at(i);
    torques_publisher_.publish(msg);
  };
  inline void setPositions() {
    std_msgs::Float32MultiArray msg;
    msg.data.resize(goal_.position.size());
    for (int i=0; i<goal_.position.size(); ++i)
      msg.data[i] = goal_.position.at(i);
    positions_publisher_.publish(msg);
  };
  inline void setTime(float time) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "{\"method\":\"setTime\",\"params\":\"[" << time
       << "],\"id\":\"1\"}";
    msg.data = ss.str();
    json_publisher_.publish(msg);
  };
  inline void setFeedback(int feedback) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "{\"method\":\"setFeedback\",\"params\":\"[" << feedback
       << "],\"id\":\"1\"}";
    msg.data = ss.str();
    json_publisher_.publish(msg);
    usleep(2000);
  };
 protected:
  void SensorTorqueCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
    c_.torque = data->data;
  }
  void SensorPositionCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
    c_.position = data->data;
  }
  void SensorDebugCallback(const std_msgs::Float32MultiArray::ConstPtr& data) {
    c_.debug = data->data;
  }
  currenter goal_;
  currenter c_;
  ros::NodeHandle nh_;
  ros::Publisher json_publisher_;
  ros::Publisher modes_publisher_;
  ros::Publisher torques_publisher_;
  ros::Publisher positions_publisher_;
  ros::Subscriber torque_subscriber_;
  ros::Subscriber position_subscriber_;
  ros::Subscriber debug_subscriber_;
};

#endif
