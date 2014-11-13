#include "ssb_common_vec.h"

namespace ssb_common_vec {

VecTentacle::VecTentacle() {
  for (int i = 0; i < 7; ++i)
    joint[i] = 0;
}

VecTentacle::VecTentacle(const float init_joint0,
                               const float init_joint1,
                               const float init_joint2,
                               const float init_joint3,
                               const float init_joint4,
                               const float init_joint5,
                               const float init_joint6) {
  joint[0] = init_joint0;
  joint[1] = init_joint1;
  joint[2] = init_joint2;
  joint[3] = init_joint3;
  joint[4] = init_joint4;
  joint[5] = init_joint5;
  joint[6] = init_joint6;
}

VecTentacle::VecTentacle(const VecTentacle &obj) {
  for (int i = 0; i < 7; ++i)
    joint[i] = obj.joint[i];
}

VecTentacle& VecTentacle::operator=(const VecTentacle &obj) {
  for (int i = 0; i < 7; ++i)
    joint[i] = obj.joint[i];
  return *this;
}

VecTentacle& VecTentacle::operator+=(const VecTentacle &obj) {
  for (int i = 0; i < 7; ++i)
    joint[i] += obj.joint[i];
  return *this;
}

VecTentacle& VecTentacle::operator-=(const VecTentacle &obj) {
  for (int i = 0; i < 7; ++i)
    joint[i] -= obj.joint[i];
  return *this;
}

VecTentacle& VecTentacle::operator/=(float kGain) {
  for (int i = 0; i < 7; ++i)
    joint[i] /= kGain;
  return *this;
}

VecTentacle& VecTentacle::operator*=(float kGain) {
  for (int i = 0; i < 7; ++i)
    joint[i] *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecEye::VecEye() {
  horizontal = 0;
  vertical = 0;
}

VecEye::VecEye(const float init_horizontal, const float init_vertical) {
  horizontal = init_horizontal;
  vertical = init_vertical;
}

VecEye::VecEye(const VecEye &obj) {
  horizontal = obj.horizontal;
  vertical = obj.vertical;
}

VecEye& VecEye::operator=(const VecEye &obj) {
  horizontal = obj.horizontal;
  vertical = obj.vertical;
  return *this;
}

VecEye& VecEye::operator+=(const VecEye &obj) {
  horizontal += obj.horizontal;
  vertical += obj.vertical;
  return *this;
}

VecEye& VecEye::operator-=(const VecEye &obj) {
  horizontal -= obj.horizontal;
  vertical -= obj.vertical;
  return *this;
}

VecEye& VecEye::operator/=(const float kGain) {
  horizontal /= kGain;
  vertical /= kGain;
  return *this;
}

VecEye& VecEye::operator*=(const float kGain) {
  horizontal *= kGain;
  vertical *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecQuadOEye::VecQuadOEye() {
  horizontal_l = 0;
  vertical_l = 0;
  horizontal_r = 0;
  vertical_r = 0;
}

VecQuadOEye::VecQuadOEye(const float init_horizontal_l, const float init_vertical_l,
                         const float init_horizontal_r, const float init_vertical_r) {
  horizontal_l = init_horizontal_l;
  vertical_l = init_vertical_l;
  horizontal_r = init_horizontal_r;
  vertical_r = init_vertical_r;
}

VecQuadOEye::VecQuadOEye(const VecQuadOEye &obj) {
  horizontal_l = obj.horizontal_l;
  vertical_l = obj.vertical_l;
  horizontal_r = obj.horizontal_r;
  vertical_r = obj.vertical_r;
}

VecQuadOEye& VecQuadOEye::operator=(const VecQuadOEye &obj) {
  horizontal_l = obj.horizontal_l;
  vertical_l = obj.vertical_l;
  horizontal_r = obj.horizontal_r;
  vertical_r = obj.vertical_r;
  return *this;
}

VecQuadOEye& VecQuadOEye::operator+=(const VecQuadOEye &obj) {
  horizontal_l += obj.horizontal_l;
  vertical_l += obj.vertical_l;
  horizontal_r += obj.horizontal_r;
  vertical_r += obj.vertical_r;
  return *this;
}

VecQuadOEye& VecQuadOEye::operator-=(const VecQuadOEye &obj) {
  horizontal_l -= obj.horizontal_l;
  vertical_l -= obj.vertical_l;
  horizontal_r -= obj.horizontal_r;
  vertical_r -= obj.vertical_r;
  return *this;
}

VecQuadOEye& VecQuadOEye::operator/=(const float kGain) {
  horizontal_l /= kGain;
  vertical_l /= kGain;
  horizontal_r /= kGain;
  vertical_r /= kGain;
  return *this;
}

VecQuadOEye& VecQuadOEye::operator*=(const float kGain) {
  horizontal_l *= kGain;
  vertical_l *= kGain;
  horizontal_r *= kGain;
  vertical_r *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecGripper::VecGripper() {
  left = 0;
  right = 0;
}

VecGripper::VecGripper(const float init_right, const float init_left) {
  left = init_left;
  right = init_right;
}

VecGripper::VecGripper(const VecGripper &obj) {
  left = obj.left;
  right = obj.right;
}

VecGripper& VecGripper::operator=(const VecGripper &obj) {
  left = obj.left;
  right = obj.right;
  return *this;
}

VecGripper& VecGripper::operator+=(const VecGripper &obj) {
  left += obj.left;
  right += obj.right;
  return *this;
}

VecGripper& VecGripper::operator-=(const VecGripper &obj) {
  left -= obj.left;
  right -= obj.right;
  return *this;
}

VecGripper& VecGripper::operator/=(const float kGain) {
  left /= kGain;
  right /= kGain;
  return *this;
}

VecGripper& VecGripper::operator*=(const float kGain) {
  left *= kGain;
  right *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecBody::VecBody() {
  joints.resize(30);
  for (int i = 0; i < joints.size(); ++i)
    joints[i] = 0;
}

VecBody::VecBody(const int init_size) {
  joints.resize(init_size);
  for (int i = 0; i < joints.size(); ++i)
    joints[i] = 0;
}

VecBody::VecBody(const std::vector<float> init_joints) {
  joints.resize(init_joints.size());
  for (int i = 0; i < joints.size(); ++i)
    joints[i] = init_joints.at(i);
}

VecBody::VecBody(const VecBody &obj) {
  if (joints.size() < obj.joints.size())
    joints.resize(obj.joints.size());
  for (int i = 0; i < obj.joints.size(); ++i)
    joints[i] = obj.joints.at(i);
}

VecBody& VecBody::operator=(const VecBody &obj) {
  if (joints.size() < obj.joints.size())
    joints.resize(obj.joints.size());
  for (int i = 0; i < obj.joints.size(); ++i)
    joints[i] = obj.joints.at(i);
  return *this;
}

VecBody& VecBody::operator+=(const VecBody &obj) {
  if (joints.size() < obj.joints.size())
    joints.resize(obj.joints.size());
  for (int i = 0; i < obj.joints.size(); ++i)
    joints[i] += obj.joints.at(i);
  return *this;
}

VecBody& VecBody::operator-=(const VecBody &obj) {
  if (joints.size() < obj.joints.size())
    joints.resize(obj.joints.size());
  for (int i = 0; i < obj.joints.size(); ++i)
    joints[i] -= obj.joints.at(i);
  return *this;
}

VecBody& VecBody::operator/=(float kGain) {
  for (int i = 0; i < joints.size(); ++i)
    joints[i] /= kGain;
  return *this;
}

VecBody& VecBody::operator*=(float kGain) {
  for (int i = 0; i < joints.size(); ++i)
    joints[i] *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecTime::VecTime() {
  start_time = 0;
  play_time = 0;
}

VecTime::VecTime(const float init_start_time, const float init_play_time) {
  start_time = init_start_time;
  play_time = init_play_time;
}

VecTime::VecTime(const VecTime &obj) {
  start_time = obj.start_time;
  play_time = obj.play_time;
}

VecTime& VecTime::operator=(const VecTime &obj) {
  start_time = obj.start_time;
  play_time = obj.play_time;
  return *this;
}

VecTime& VecTime::operator+=(const VecTime &obj) {
  start_time += obj.start_time;
  play_time += obj.play_time;
  return *this;
}

VecTime& VecTime::operator-=(const VecTime &obj) {
  start_time -= obj.start_time;
  play_time -= obj.play_time;
  return *this;
}

VecTime& VecTime::operator/=(float kGain) {
  start_time /= kGain;
  play_time /= kGain;
  return *this;
}

VecTime& VecTime::operator*=(float kGain) {
  start_time *= kGain;
  play_time *= kGain;
  return *this;
}

//---------------------------------------------------------------

VecEvent::VecEvent() {
  msg = "quit";
  start_time = 0.0;
  end_time = 0.0;
  args = "\0";
}

VecEvent::VecEvent(const std::string init_msg,
                   const float init_start_time, const float init_end_time,
                   const std::string init_args) {
  msg = init_msg;
  start_time = init_start_time;
  end_time = init_end_time;
  args = init_args;
}

VecEvent::VecEvent(const VecEvent &obj) {
  msg = obj.msg;
  start_time = obj.start_time;
  end_time = obj.end_time;
  args = obj.args;
}

VecEvent& VecEvent::operator=(const VecEvent &obj) {
  msg = obj.msg;
  start_time = obj.start_time;
  end_time = obj.end_time;
  args = obj.args;
  return *this;
}

// If you simply want to shift the start and end time, add a VecTime.
// The adding msg operation is rather a partial clip extarcting operation.
// You can extract parts of a msg using this operator.
// Note that the operator is not a "*." since extracting is only one of the
// facets of this operator.
// The operator can also be  used in the sequence of moving through partial clips.
// For instance in a loop of
// A += B("same msg as A", clip size, clip size, args)
// will keep on shifting to the next clip in each loop.
VecEvent& VecEvent::operator+=(const VecEvent &obj) {
  start_time += obj.start_time;
  end_time = start_time + obj.end_time - obj.start_time;
  return *this;
}

VecEvent& VecEvent::operator-=(const VecEvent &obj) {
  start_time -= obj.start_time;
  end_time = start_time + obj.end_time - obj.start_time;
  return *this;
}

VecEvent& VecEvent::operator+=(const VecTime &obj) {
  start_time += obj.start_time;
  end_time += obj.start_time + obj.play_time;
  return *this;
}

VecEvent& VecEvent::operator*=(const VecTime &obj) {
  end_time = start_time*obj.start_time + obj.play_time*(end_time - start_time);
  start_time *= obj.start_time;
  return *this;
}

// This operation just minimizes the play_time of the msg.
VecEvent& VecEvent::operator/=(float kGain) {
  end_time = start_time + (end_time - start_time) / kGain;
  return *this;
}

VecEvent& VecEvent::operator*=(float kGain) {
  end_time = start_time + (end_time - start_time) * kGain;
  return *this;
}

//---------------------------------------------------------------

VecVoice::VecVoice() {
  msg = "quit";
  start_time = 0.0;
  end_time = 0.0;
  args = "1.0";
}

VecVoice::VecVoice(const std::string init_msg,
                   const float init_start_time, const float init_end_time,
                   const std::string init_args) {
  msg = init_msg;
  start_time = init_start_time;
  end_time = init_end_time;
  args = init_args;
}

VecVoice::VecVoice(const VecVoice &obj) {
  msg = obj.msg;
  start_time = obj.start_time;
  end_time = obj.end_time;
  args = obj.args;
}

//---------------------------------------------------------------

VecInterpolation::VecInterpolation() {
  type = 0;
  p.clear();
}

VecInterpolation::VecInterpolation(const int init_type, const std::vector<ofVec2f> init_p) {
  type = init_type;
  p.clear();
  p.resize(init_p.size());
  for (int i = 0; i < p.size(); ++i)
    p.at(i) = init_p.at(i);
}

VecInterpolation::VecInterpolation(const VecInterpolation &obj) {
  type = obj.type;
  p.clear();
  p.resize(obj.p.size());
  for (int i = 0; i < p.size(); ++i)
    p.at(i) = obj.p.at(i);
}

} // namespace ssb_common_vec
