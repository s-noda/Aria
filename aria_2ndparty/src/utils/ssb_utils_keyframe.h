#ifndef SSB_UTILS_KEYFRAME_H_
#define SSB_UTILS_KEYFRAME_H_

#include "ssb_common_common.h"
#include "ssb_utils_interpolation.h"

// Code description on keyframe:
// KeyFrameCanvas/KeyFramePlayer behave differently with VecEvent types.
// Therefore, these two classes use a specialization trick to
// specify vector classes derived from VecEvent.
// In this way, there should be no need for creating new specializations
// when adding a new class derived from VecEvent.
// On the other hand, the KeyFrameEditor usually behaves differently
// among events.
// For example, a voice's play time or video should be automatically
// edited with its given play time from the wav file.
// But, a projection's play time has no given time nor has an end.
// Therefore, the template is specified among classes.

// What's typical with VecEvent derived classes?
//  1. VecEvent classes do not interpolate. It is an on and off switch.
//  2. VecEvent classes pause by command, not by stopping calculation.

// Usage warning on keyframe:
// In general, KeyFrame types should be specified with the type of event,
// if the template type is a derived class of VecEvent.
//   OK:  KeyFrame<VecVoice> voice_key_frame;
//   BAD: KeyFrame<VecEvent> voice_key_frame;
// This is because file loading and saving classes (e.g. xml classes) depend on
// specific types. Also, defualt constructors(zero amount) can differ between events.


namespace ssb_utils_keyframe {

template<typename T> class KeyFrame {
 public:
  KeyFrame() {};
  KeyFrame(T p, ssb_common_vec::VecTime t, ssb_common_vec::VecInterpolation i) {
    part_data_ = p;
    time_data_ = t;
    interpolation_data_ = i;
  };
  ~KeyFrame() {};
  bool operator<(const KeyFrame &kf) const {
    return (time_data_.start_time < kf.time_data_.start_time);
  }
  void set_part_data(T part_data) { part_data_ = part_data; };
  void set_time_data(ssb_common_vec::VecTime time_data) { time_data_ = time_data; };
  void set_interpolation_data(ssb_common_vec::VecInterpolation interpolation_data) {
    interpolation_data_ = interpolation_data;
  };
  T get_part_data() { return part_data_; };
  ssb_common_vec::VecTime get_time_data() { return time_data_; };
  ssb_common_vec::VecInterpolation get_interpolation_data() { return interpolation_data_; };
 private:
  T part_data_;
  ssb_common_vec::VecTime time_data_;
  ssb_common_vec::VecInterpolation interpolation_data_;
};

template<typename T, typename T_model> class KeyFramePlayer {
 public:
  KeyFramePlayer(std::string tname, std::vector<KeyFrame<T> > &kf, T_model &ref_model, int &fps);
  ~KeyFramePlayer() {};
  void Setup();
  void StartPlay();
  void OnPlay();
  inline void Play2Pause() { _Play2Pause((T*)0); };
  inline void Pause2Play() { _Pause2Play((T*)0); };
  void EndPlay();
  void PlayFrame(int frame_number) { _PlayFrame((T*)0, frame_number); };
 private:
  std::vector<ssb_common_vec::VecSequence<T> > sequence_;
  int frame_count_from_start_of_current_sequence_;
  int id_of_current_sequence_;
  ssb_utils_interpolation::Interpolation interpolation_function_;
  T send_value_;
  bool play_;
  int &frames_per_sec;
  // Save target addresses in order to keep structures similar to KeyFrameEditor.
  std::vector<KeyFrame<T> > &key_list;
  T_model &model;
  void _Play2Pause(void*);
  void _Play2Pause(ssb_common_vec::VecEvent*);
  void _Pause2Play(void*);
  void _Pause2Play(ssb_common_vec::VecEvent*);
  void _PlayFrame(void*, int frame_number);
  void _PlayFrame(ssb_common_vec::VecEvent*, int frame_number);
};

} // namespace ssb_utils_keyframe

#endif
