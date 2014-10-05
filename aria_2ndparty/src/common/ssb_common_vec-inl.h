#ifndef SSB_COMMON_VEC_INL_H_
#define SSB_COMMON_VEC_INL_H_

#include "ssb_common_vec.h"

namespace ssb_common_vec {

template<typename T>
VecSequence<T>::VecSequence() {
  number_of_frames = 0;
  is_interpolation = false;
  interpolation_type = 0;
  key_ref = 0;
  diff = T();
}

template<typename T>
VecSequence<T>::VecSequence(const int _number_of_frames,
                         const bool _is_interpolation,
                         const int _interpolation_type,
                         const int _key_ref,
                         const T _diff) {
  number_of_frames = _number_of_frames;
  is_interpolation = _is_interpolation;
  interpolation_type = _interpolation_type;
  key_ref = _key_ref;
  diff = _diff;
}
                         
template<typename T>
VecSequence<T>::VecSequence(const VecSequence<T> &obj) {
  number_of_frames = obj.number_of_frames;
  is_interpolation = obj.is_interpolation;
  interpolation_type = obj.interpolation_type;
  key_ref = obj.key_ref;
  diff = obj.diff;
}

template<typename T>
VecSequence<T>& VecSequence<T>::operator=(const VecSequence<T> &obj) {
  number_of_frames = obj.number_of_frames;
  is_interpolation = obj.is_interpolation;
  interpolation_type = obj.interpolation_type;
  key_ref = obj.key_ref;
  diff = obj.diff;
  return *this;
}

} // namespace ssb_common_vec

#endif
