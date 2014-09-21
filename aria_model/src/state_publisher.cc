#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

class AriaToJointState {
public:
  explicit AriaToJointState(ros::NodeHandle nh);
  ~AriaToJointState() {};
private:
  void SensorCallback(const std_msgs::Float32MultiArray msg);
  ros::NodeHandle nh_;
  ros::Publisher joint_state_publisher_;
  ros::Subscriber currentor_subscriber_;
};


AriaToJointState::AriaToJointState(ros::NodeHandle nh) {
  nh_ = nh;
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  currentor_subscriber_ = nh_.subscribe("/currentor_socket/sensor_array/position", 1, &AriaToJointState::SensorCallback, this);
}

void AriaToJointState::SensorCallback(const std_msgs::Float32MultiArray msg) {
  ROS_INFO("callback");
  if (msg.data.size() < 22)
    return;
  ROS_INFO("got sensor");
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(22);
  joint_state.position.resize(22);
  joint_state.name[0] = "hip2";
  joint_state.position[0] = msg.data[18];    
  joint_state.name[1] = "hip3";
  joint_state.position[1] = 0.5*msg.data[19];    
  joint_state.name[2] = "body";
  joint_state.position[2] = -0.5*msg.data[20];
  joint_state.name[3] = "arm_r_joint1";
  joint_state.position[3] = msg.data[1];
  joint_state.name[4] = "arm_r_joint2";
  joint_state.position[4] = msg.data[2];
  joint_state.name[5] = "arm_r_joint3";
  joint_state.position[5] = msg.data[3];
  joint_state.name[6] = "arm_r_joint4";
  joint_state.position[6] = -msg.data[4];
  joint_state.name[7] = "arm_r_joint5";
  joint_state.position[7] = -msg.data[5];
  joint_state.name[8] = "arm_r_joint6";
  joint_state.position[8] = -msg.data[6];
  joint_state.name[9] = "arm_r_joint7";
  joint_state.position[9] = -msg.data[7];
  joint_state.name[10] = "arm_r_hand1";
  joint_state.position[10] = msg.data[0];
  joint_state.name[11] = "arm_l_joint1";
  joint_state.position[11] = msg.data[8];
  joint_state.name[12] = "arm_l_joint2";
  joint_state.position[12] = msg.data[9];
  joint_state.name[13] = "arm_l_joint3";
  joint_state.position[13] = msg.data[10];
  joint_state.name[14] = "arm_l_joint4";
  joint_state.position[14] = -msg.data[11];
  joint_state.name[15] = "arm_l_joint5";
  joint_state.position[15] = msg.data[12];
  joint_state.name[16] = "arm_l_joint6";
  joint_state.position[16] = -msg.data[13];
  joint_state.name[17] = "arm_l_joint7";
  joint_state.position[17] = msg.data[14];
  joint_state.name[18] = "arm_l_hand1";
  joint_state.position[18] = msg.data[0];
  joint_state.name[19] = "neck1";
  joint_state.position[19] = msg.data[15];
  joint_state.name[20] = "neck2";
  joint_state.position[20] = msg.data[16];
  joint_state.name[21] = "neck3";
  joint_state.position[21] = msg.data[17];
  joint_state_publisher_.publish(joint_state);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle nh;
  AriaToJointState aria(nh);
  ros::Rate loop_rate(30);
  ROS_INFO("okay");
  while (ros::ok()) {
    ROS_INFO("debug");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
