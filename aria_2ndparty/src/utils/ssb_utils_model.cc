#include "ssb_utils_model.h"

namespace ssb_utils_model {

void TentacleModel::setParams(ssb_common_enum::Config settings) {
  ssb_common_vec::VecTentacle min_input_settings(-45, -45, -45, -45, -45, -45, -45);
  ssb_common_vec::VecTentacle max_input_settings(45, 45, 45, 45, 45, 45, 45);
  ssb_common_vec::VecTentacle min_output_settings(1100, 1100, 1286, 1240, 1215, 1000, 1000);
  ssb_common_vec::VecTentacle max_output_settings(1900, 1900, 1748, 1800, 1710, 2000, 2000);
  ssb_common_vec::VecTentacle home_output_settings(1500, 1500, 1500, 1400, 1500, 1500, 1500);
  ssb_common_vec::VecTentacle direction_settings(-1, 1, 1, -1, 1, -1, -1);
  min_limit_input_ = min_input_settings;
  max_limit_input_ = max_input_settings;
  min_limit_output_ = min_output_settings;
  max_limit_output_ = max_output_settings;
  home_output_ = home_output_settings;
  sgn_direction_ = direction_settings;
  for (int i = 0; i < 7; ++i)
    k_input2output_.joint[i] = (max_limit_output_.joint[i] - min_limit_output_.joint[i])
                               / (max_limit_input_.joint[i] - min_limit_input_.joint[i]);
  object_.set_motor_id(ssb_common_vec::VecTentacle(1, 2, 4, 0, 3, 5, 6));
}

void TentacleModel::Input2Output() {
  ssb_common_vec::VecTentacle tmp_output;
  for (int i = 0; i < 7; ++i) {
    // Normalize input to output direction.
    tmp_output.joint[i] = sgn_direction_.joint[i] * input_.joint[i];
    // Convert input to local amount.
    tmp_output.joint[i] -= min_limit_input_.joint[i];
    // Convert local input to local output.
    tmp_output.joint[i] *= k_input2output_.joint[i];
    // Convert output to absolute amount.
    tmp_output.joint[i] += min_limit_output_.joint[i];
    // In case output is out of range.
    if (tmp_output.joint[i] > max_limit_output_.joint[i])
      tmp_output.joint[i] = max_limit_output_.joint[i];
    else if (tmp_output.joint[i] < min_limit_output_.joint[i])
      tmp_output.joint[i] = min_limit_output_.joint[i];
  }
  output_ = tmp_output;
}

ssb_common_vec::VecTentacle TentacleModel::getHomeAsInput() {
  ssb_common_vec::VecTentacle tmp_home_as_input;
  tmp_home_as_input = home_output_;
  for (int i = 0; i < 7; ++i) {
    tmp_home_as_input.joint[i] -= min_limit_output_.joint[i];
    tmp_home_as_input.joint[i] /= k_input2output_.joint[i];
    tmp_home_as_input.joint[i] += min_limit_input_.joint[i];
    tmp_home_as_input.joint[i] *= sgn_direction_.joint[i];
  }
  return tmp_home_as_input;
}


void QuadOEyeModel::setParams(ssb_common_enum::Config settings) {
  ssb_common_vec::VecEye min_input_settings(-1.0, -1.0);
  min_limit_input_ = min_input_settings;
  ssb_common_vec::VecEye max_input_settings(1.0, 1.0);
  max_limit_input_ = max_input_settings;
  ssb_common_vec::VecQuadOEye min_output_settings(1270, 1450, 1650, 1450);
  min_limit_output_ = min_output_settings;
  ssb_common_vec::VecQuadOEye max_output_settings(1450, 1550, 1850, 1550);
  max_limit_output_ = max_output_settings;
  ssb_common_vec::VecQuadOEye home_output_settings(1350, 1500, 1750, 1480);
  home_output_ = home_output_settings;
  ssb_common_vec::VecQuadOEye direction_settings(1, 1, 1, -1);
  sgn_direction_ = direction_settings;
  object_.set_motor_id(ssb_common_vec::VecQuadOEye(0, 2, 1, 3));
  k_input2output4positive_input_ = max_limit_output_;
  k_input2output4positive_input_ -= home_output_;
  k_input2output4negative_input_ = home_output_;
  k_input2output4negative_input_ -= min_limit_output_;
}

void QuadOEyeModel::Input2Output() {
  ssb_common_vec::VecQuadOEye tmp_output;
  // Normalize input to output direction.
  tmp_output.horizontal_l = sgn_direction_.horizontal_l * input_.horizontal;
  tmp_output.vertical_l = sgn_direction_.vertical_l * input_.vertical;
  tmp_output.horizontal_r = sgn_direction_.horizontal_r * input_.horizontal;
  tmp_output.vertical_r = sgn_direction_.vertical_r * input_.vertical;
  // Convert input to local output.
  tmp_output.horizontal_l *= input_.horizontal > 0 ?
                           k_input2output4positive_input_.horizontal_l
                           : k_input2output4negative_input_.horizontal_l;
  tmp_output.vertical_l *= input_.vertical > 0 ?
                           k_input2output4positive_input_.vertical_l
                           : k_input2output4negative_input_.vertical_l;
  tmp_output.horizontal_r *= input_.horizontal > 0 ?
                           k_input2output4positive_input_.horizontal_r
                           : k_input2output4negative_input_.horizontal_r;
  tmp_output.vertical_r *= input_.vertical > 0 ?
                           k_input2output4positive_input_.vertical_r
                           : k_input2output4negative_input_.vertical_r;
  // Convert output to absolute amount.
  tmp_output.horizontal_l += home_output_.horizontal_l;
  tmp_output.vertical_l += home_output_.vertical_l;
  tmp_output.horizontal_r += home_output_.horizontal_r;
  tmp_output.vertical_r += home_output_.vertical_r;
  // In case output is out of range.
  if (tmp_output.horizontal_l > max_limit_output_.horizontal_l)
    tmp_output.horizontal_l = max_limit_output_.horizontal_l;
  else if (tmp_output.horizontal_l < min_limit_output_.horizontal_l)
    tmp_output.horizontal_l = min_limit_output_.horizontal_l;
  if (tmp_output.vertical_l > max_limit_output_.vertical_l)
    tmp_output.vertical_l = max_limit_output_.vertical_l;
  else if (tmp_output.vertical_l < min_limit_output_.vertical_l)
    tmp_output.vertical_l = min_limit_output_.vertical_l;
  if (tmp_output.horizontal_r > max_limit_output_.horizontal_r)
    tmp_output.horizontal_r = max_limit_output_.horizontal_r;
  else if (tmp_output.horizontal_r < min_limit_output_.horizontal_r)
    tmp_output.horizontal_r = min_limit_output_.horizontal_r;
  if (tmp_output.vertical_r > max_limit_output_.vertical_r)
    tmp_output.vertical_r = max_limit_output_.vertical_r;
  else if (tmp_output.vertical_r < min_limit_output_.vertical_r)
    tmp_output.vertical_r = min_limit_output_.vertical_r;
  output_ = tmp_output;
}

ssb_common_vec::VecEye QuadOEyeModel::getHomeAsInput() {
  return ssb_common_vec::VecEye(0, 0);
}


void GripperModel::setParams(ssb_common_enum::Config settings) {
  ssb_common_vec::VecGripper direction_settings(1, -1);
  sgn_direction_ = direction_settings;
}

void GripperModel::Input2Output() {
  ssb_common_vec::VecGripper tmp_output;
  tmp_output.left = sgn_direction_.left * input_.left;
  tmp_output.right = sgn_direction_.right * input_.right;
  output_ = tmp_output;
}

ssb_common_vec::VecGripper GripperModel::getHomeAsInput() {
  return ssb_common_vec::VecGripper(0, 0);
}

} // namespace ssb_utils_model
