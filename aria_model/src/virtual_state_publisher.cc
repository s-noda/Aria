#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

class AriaToJointState {
public:
  explicit AriaToJointState(ros::NodeHandle nh);
  ~AriaToJointState() {};
  void Main();
private:
  void Init();
  void SensorCallback(const std_msgs::Float32MultiArray msg);
  void GripperCallback(const std_msgs::Float32MultiArray msg);
  void EyeCallback(const std_msgs::Float32MultiArray msg);
  ros::NodeHandle nh_;
  ros::Publisher joint_state_publisher_;
  ros::Subscriber currentor_subscriber_;
  ros::Subscriber gripper_subscriber_;
  ros::Subscriber eye_subscriber_;
  sensor_msgs::JointState joint_state_;
};


AriaToJointState::AriaToJointState(ros::NodeHandle nh) {
  nh_ = nh;
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  currentor_subscriber_ = nh_.subscribe("/currentor_socket/virtual_array/position", 1, &AriaToJointState::SensorCallback, this);
  gripper_subscriber_ = nh_.subscribe("/2ndparty/virtual_array/gripper", 1, &AriaToJointState::GripperCallback, this);
  eye_subscriber_ = nh_.subscribe("/2ndparty/virtual_array/eye", 1, &AriaToJointState::EyeCallback, this);
  Init();
}


void AriaToJointState::Init() {
  joint_state_.name.resize(26);
  joint_state_.position.resize(26);
  joint_state_.name[0] = "hip2";
  joint_state_.name[1] = "hip3";
  joint_state_.name[2] = "body";
  joint_state_.name[3] = "arm_r_joint1";
  joint_state_.name[4] = "arm_r_joint2";
  joint_state_.name[5] = "arm_r_joint3";
  joint_state_.name[6] = "arm_r_joint4";
  joint_state_.name[7] = "arm_r_joint5";
  joint_state_.name[8] = "arm_r_joint6";
  joint_state_.name[9] = "arm_r_joint7";
  joint_state_.name[10] = "arm_r_hand1";
  joint_state_.name[11] = "arm_l_joint1";
  joint_state_.name[12] = "arm_l_joint2";
  joint_state_.name[13] = "arm_l_joint3";
  joint_state_.name[14] = "arm_l_joint4";
  joint_state_.name[15] = "arm_l_joint5";
  joint_state_.name[16] = "arm_l_joint6";
  joint_state_.name[17] = "arm_l_joint7";
  joint_state_.name[18] = "arm_l_hand1";
  joint_state_.name[19] = "neck1";
  joint_state_.name[20] = "neck2";
  joint_state_.name[21] = "neck3";
  joint_state_.name[22] = "eye_r_hori";
  joint_state_.name[23] = "eye_r_vert";
  joint_state_.name[24] = "eye_l_hori";
  joint_state_.name[25] = "eye_l_vert";
}

void AriaToJointState::SensorCallback(const std_msgs::Float32MultiArray msg) {
  if (msg.data.size() < 22)
    return;
  joint_state_.position[0] = msg.data[18];
  joint_state_.position[1] = 0.5*msg.data[19];    
  joint_state_.position[2] = -0.5*msg.data[20];
  joint_state_.position[3] = msg.data[1];
  joint_state_.position[4] = msg.data[2];
  joint_state_.position[5] = msg.data[3];
  joint_state_.position[6] = -msg.data[4];
  joint_state_.position[7] = -msg.data[5];
  joint_state_.position[8] = -msg.data[6];
  joint_state_.position[9] = -msg.data[7];
  joint_state_.position[11] = msg.data[8];
  joint_state_.position[12] = msg.data[9];
  joint_state_.position[13] = msg.data[10];
  joint_state_.position[14] = -msg.data[11];
  joint_state_.position[15] = msg.data[12];
  joint_state_.position[16] = -msg.data[13];
  joint_state_.position[17] = msg.data[14];
  joint_state_.position[19] = msg.data[15];
  joint_state_.position[20] = msg.data[16];
  joint_state_.position[21] = msg.data[17];
}

void AriaToJointState::GripperCallback(const std_msgs::Float32MultiArray msg) {
  if (msg.data.size() < 2)
    return;
  joint_state_.position[10] = msg.data[0];
  joint_state_.position[18] = msg.data[1];
}

void AriaToJointState::EyeCallback(const std_msgs::Float32MultiArray msg) {
  if (msg.data.size() < 2)
    return;
  joint_state_.position[22] = msg.data[0];
  joint_state_.position[23] = msg.data[1];
  joint_state_.position[24] = msg.data[0];
  joint_state_.position[25] = msg.data[1];
}

void AriaToJointState::Main() {
  joint_state_.header.stamp = ros::Time::now();
  joint_state_publisher_.publish(joint_state_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle nh;
  AriaToJointState aria(nh);
  ros::Rate loop_rate(30);
  ROS_INFO("okay");
  while (ros::ok()) {
    aria.Main();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
