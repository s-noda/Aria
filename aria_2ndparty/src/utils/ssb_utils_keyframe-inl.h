#ifndef SSB_UTILS_KEYFRAME_INL_H_
#define SSB_UTILS_KEYFRAME_INL_H_

#include "ssb_utils_keyframe.h"

namespace ssb_utils_keyframe {

template<typename T, typename T_model>
  KeyFramePlayer<T, T_model>::KeyFramePlayer(std::string tname, std::vector<KeyFrame<T> > &kf, T_model &ref_model, int &fps) : key_list(kf), model(ref_model), frames_per_sec(fps) {
  play_ = false;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::Setup() {
  if (key_list.size() == 0)
    return;
  sequence_.clear();
  sequence_.resize(key_list.size()*2-1);
  T diff_0; // The default initializer initializes any typename T with zero.
  sequence_.at(0) = ssb_common_vec::VecSequence<T>(
      key_list.at(0).get_time_data().play_time*frames_per_sec, // number_of_frames
      false, // is_interpolation
      0, // interpolation_type
      0, // key_ref
      diff_0);
  for (int i = 1; i < key_list.size(); ++i) {
    T diff = key_list.at(i).get_part_data();
    diff -= key_list.at(i-1).get_part_data();
    sequence_.at(2*i-1) = ssb_common_vec::VecSequence<T>(
        (key_list.at(i).get_time_data().start_time
         - key_list.at(i-1).get_time_data().start_time
         - key_list.at(i-1).get_time_data().play_time)*frames_per_sec,
        true,
        key_list.at(i-1).get_interpolation_data().type,
        i-1,
        diff);
    sequence_.at(2*i) = ssb_common_vec::VecSequence<T>(
        key_list.at(i).get_time_data().play_time*frames_per_sec,
        false,
        0,
        i,
        diff_0);
  }
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  interpolation_function_.set_interpolation(key_list.at(0).get_interpolation_data());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::StartPlay() {
  if (key_list.size() == 0)
    return;
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  model.set_input(send_value_);
  model.Input2Output();
  model.send();
  interpolation_function_.set_interpolation(key_list.at(0).get_interpolation_data());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
  play_ = true;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::OnPlay() {
  if (!play_)
    return;
  frame_count_from_start_of_current_sequence_++;
  if (frame_count_from_start_of_current_sequence_
      > sequence_.at(id_of_current_sequence_).number_of_frames) {
    id_of_current_sequence_++;
    frame_count_from_start_of_current_sequence_ = 0;
    if (id_of_current_sequence_ > (sequence_.size() - 1)) {
      EndPlay();
      return;
    } else if (sequence_.at(id_of_current_sequence_).is_interpolation == false) {
      send_value_ =
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
      model.set_input(send_value_);
      model.Input2Output();
      model.send();
      // Set the interpolation function for next sequence.
      interpolation_function_.set_interpolation(
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_interpolation_data());
      return;
    }
  } else if (sequence_.at(id_of_current_sequence_).interpolation_type > 0) {
    send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
    T add_amount = sequence_.at(id_of_current_sequence_).diff;
    add_amount *= interpolation_function_.F(
        static_cast<float>(frame_count_from_start_of_current_sequence_)
        /sequence_.at(id_of_current_sequence_).number_of_frames);
    send_value_ += add_amount;
    model.set_input(send_value_);
    model.Input2Output();
    model.send();
  }
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_Play2Pause(void*) {
  if (key_list.size() == 0)
    return;
  play_ = false;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_Pause2Play(void*) {
  if (key_list.size() == 0)
    return;
  play_ = true;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::EndPlay() {
  if (key_list.size() == 0)
    return;
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  interpolation_function_.set_interpolation(ssb_common_vec::VecInterpolation());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
  play_ = false;
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_PlayFrame(void*, int frame_number) {
  if (key_list.size() == 0)
    return;
  int frame_count_to_find_current_sequence = 0;
  for (int i = 0; i < (sequence_.size()-1); ++i) {
    frame_count_to_find_current_sequence += sequence_.at(i).number_of_frames;
    if (frame_number <= frame_count_to_find_current_sequence) {
      id_of_current_sequence_ = i;
      send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref)
                    .get_part_data();
      frame_count_from_start_of_current_sequence_ =
          frame_number
          - (frame_count_to_find_current_sequence - sequence_.at(i).number_of_frames);
      interpolation_function_.set_interpolation(
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_interpolation_data());
      T add_amount = sequence_.at(id_of_current_sequence_).diff;
      add_amount *= interpolation_function_.F(
          static_cast<float>(frame_count_from_start_of_current_sequence_)
          /sequence_.at(id_of_current_sequence_).number_of_frames);
      send_value_ += add_amount;
      model.set_input(send_value_);
      model.Input2Output();
      model.send();
      return;
    }
  }
  // If frame_number is out of play range.
  send_value_ = key_list.at(key_list.size()-1).get_part_data();
  model.set_input(send_value_);
  model.Input2Output();
  model.send();
}

//---------------------------------------------------------------

// Specialized functions for event.
// Events are not dynamic motions but rather a on/off trigger.
// Therefore the pause function works differently.
template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_Play2Pause(ssb_common_vec::VecEvent*) {
  if (key_list.size() == 0)
    return;
  play_ = false;
  send_value_ = ssb_common_vec::VecVoice();
  model.set_input(send_value_);
  model.Input2Output();
  model.send();
}

template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_Pause2Play(ssb_common_vec::VecEvent*) {
  if (key_list.size() == 0)
    return;
  play_ = true;
  if (key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data().msg == "quit")
    return;
  send_value_ = ssb_common_vec::VecVoice("continue", 0, -1, "1.0");
  model.set_input(send_value_);
  model.Input2Output();
  model.send();
}

// Events will play nothing during the default interpolation.
// The following method allows non-interpolated events to be played at frame level,
// or to be more accurate, as if played at frame level.
template<typename T, typename T_model>
void KeyFramePlayer<T, T_model>::_PlayFrame(ssb_common_vec::VecEvent*, int frame_number) {
  if (key_list.size() == 0)
    return;
  int frame_count_to_find_current_sequence = 0;
  for (int i = 0; i < sequence_.size(); ++i) {
    frame_count_to_find_current_sequence += sequence_.at(i).number_of_frames;
    if (frame_number <= frame_count_to_find_current_sequence) {
      id_of_current_sequence_ = i;
      frame_count_from_start_of_current_sequence_ =
          frame_number
          - (frame_count_to_find_current_sequence - sequence_.at(i).number_of_frames);
      if (sequence_.at(id_of_current_sequence_).is_interpolation)
        return;
      send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
      if (send_value_.msg == "quit")
        return;
      ssb_common_vec::VecVoice add_amount = send_value_;
      add_amount /= sequence_.at(i).number_of_frames;
      add_amount += ssb_common_vec::VecTime(add_amount.end_time, 0);      
      // expand play time
      add_amount *= frame_count_from_start_of_current_sequence_;
      // shift start time to current frame but keep end time the same
      add_amount += ssb_common_vec::VecTime(add_amount.end_time-2*add_amount.start_time,
                                            -add_amount.end_time+2*add_amount.start_time);
      // adjust play length of chunk
      add_amount *= ssb_common_vec::VecTime(1.0, 10.0);
      send_value_ = add_amount;
      model.set_input(send_value_);
      model.Input2Output();
      model.send();
      return;
    }
  }
  // If frame_number is out of play range or is a non-speech("quit") frame.
  send_value_ = ssb_common_vec::VecVoice();
  model.set_input(send_value_);
  model.Input2Output();
  model.send();
}


} // namespace ssb_utils_keyframe

#endif
