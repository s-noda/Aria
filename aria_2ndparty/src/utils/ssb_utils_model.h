#ifndef SSB_UTILS_MODEL_H_
#define SSB_UTILS_MODEL_H_

#include "ssb_common_common.h"
#include "ssb_actuators_kduinoservo.h"
#include "ssb_actuators_dynamixel.h"
#include "ssb_actuators_virtual.h"
#include "ssb_sensors_voice.h"
#include "virtual_aria.h"

namespace ssb_utils_model {
  
template<typename T_input, typename T_output, typename T_real> class Model {
 public:
  explicit Model(ros::NodeHandle &nh) : object_(nh) {};
  virtual ~Model() {};
  virtual void setParams(ssb_common_enum::Config settings) = 0;
  virtual void Input2Output() = 0;
  void send() { object_.publish(output_); };
  void set_output(T_output output) { output_ = output; };
  void set_input(T_input input) { input_ = input; };
  T_output get_output() { return output_; };
  T_input get_input() { return input_; };
  virtual T_input getHomeAsInput() = 0;
 protected:
  // Keeps input value from the UI.
  T_input input_;
  T_input min_limit_input_, max_limit_input_;
  // Exchanged value from input, that is kept for sending to real objects.
  // Or directly applied output values that are kept to be sent.
  T_output output_;
  T_output min_limit_output_, max_limit_output_;
  // Keep starting home output state value of real objects.
  T_output home_output_;
  // While the ui input might be clockwise the real output might be counter clockwise.
  // The intuitive input directions might not correspond to the actual output directions.
  // Set -1 for non-corresponding outputs and 1 for corresponding outputs.
  T_output sgn_direction_;
  T_real object_;
};


class TentacleModel : public Model<ssb_common_vec::VecTentacle,
                                   ssb_common_vec::VecTentacle,
                                   ssb_actuators_kduinoservo::TentacleServo> {
 public:
  explicit TentacleModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~TentacleModel() {};
  void setParams(ssb_common_enum::Config settings);
  void Input2Output();
  ssb_common_vec::VecTentacle getHomeAsInput();
 private:
  // Convertion gain for exchanging input values to output values.
  ssb_common_vec::VecTentacle k_input2output_;
};


class QuadOEyeModel : public Model<ssb_common_vec::VecEye,
                                   ssb_common_vec::VecQuadOEye,
                                   ssb_actuators_kduinoservo::QuadOEyeServo> {
 public:
  explicit QuadOEyeModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~QuadOEyeModel() {};
  void setParams(ssb_common_enum::Config settings);
  void Input2Output();
  ssb_common_vec::VecEye getHomeAsInput();
 private:
  ssb_common_vec::VecQuadOEye k_input2output4positive_input_;
  ssb_common_vec::VecQuadOEye k_input2output4negative_input_;
};


class VirtualQuadOEyeModel : public Model<ssb_common_vec::VecEye,
                                          ssb_common_vec::VecQuadOEye,
                                          ssb_actuators_virtual::VirtualQuadOEyeObject> {
 public:
  explicit VirtualQuadOEyeModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~VirtualQuadOEyeModel() {};
  void setParams(ssb_common_enum::Config settings);
  void Input2Output();
  ssb_common_vec::VecEye getHomeAsInput();
 private:
  ssb_common_vec::VecQuadOEye k_input2output4positive_input_;
  ssb_common_vec::VecQuadOEye k_input2output4negative_input_;
};


class GripperModel : public Model<ssb_common_vec::VecGripper,
                                  ssb_common_vec::VecGripper,
                                  ssb_actuators_dynamixel::GripperDynamixel> {
 public:
  explicit GripperModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~GripperModel() {};
  void setParams(ssb_common_enum::Config settings);
  void Input2Output();
  ssb_common_vec::VecGripper getHomeAsInput();
 private:
  ssb_common_vec::VecGripper k_input2output_;
};


class VirtualGripperModel : public Model<ssb_common_vec::VecGripper,
                                         ssb_common_vec::VecGripper,
                                         ssb_actuators_virtual::VirtualGripperObject> {
 public:
  explicit VirtualGripperModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~VirtualGripperModel() {};
  void setParams(ssb_common_enum::Config settings);
  void Input2Output();
  ssb_common_vec::VecGripper getHomeAsInput();
 private:
  ssb_common_vec::VecGripper k_input2output_;
};


class VirtualAriaModel : public Model<ssb_common_vec::VecBody,
                                      ssb_common_vec::VecBody,
                                      aria::VirtualAria> {
 public:
  explicit VirtualAriaModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  };
  ~VirtualAriaModel() {};
  void setParams(ssb_common_enum::Config settings) {};
  void Input2Output() { output_ = input_; };
  ssb_common_vec::VecBody getHomeAsInput() { return ssb_common_vec::VecBody(30); };
};

class VoiceModel : public Model<ssb_common_vec::VecVoice,
                                ssb_common_vec::VecVoice,
                                ssb_media_event::Voice> {
 public:
  explicit VoiceModel(ros::NodeHandle &nh) : Model(nh) {
    setParams(ssb_common_enum::DEBUG);
  }
  ~VoiceModel() {};
  void setParams(ssb_common_enum::Config settings) {};
  void Input2Output() { output_ = input_; };
  ssb_common_vec::VecVoice getHomeAsInput() { return ssb_common_vec::VecVoice(); };
};

} // namespace ssb_utils_model

#endif
