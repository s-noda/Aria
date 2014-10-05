#ifndef SSB_UTILS_INTERPOLATION_H_
#define SSB_UTILS_INTERPOLATION_H_

#include "ssb_common_common.h"

namespace ssb_utils_interpolation {

// Interpolation interpolates in six ways.
// Call set_type to set the interpolation mode.
// Call Init to initialize the interpolation function.
// Call B to get the y-value of interpolation at time t.
// You can also edit the interpolation function using set_p.
// The editing style can be set using the set_set_p.
// This will allows you to have multiple editing styles for
// each interpolation type if necessary.
// Note, the Interpolation class does not have a
// VecInterpolation member since the class directly
// handles interpolation points and using a VecInterpolation
// member can make the code look complex.
// Although the class can return a VecInterpolation type.
class Interpolation {
 public:
  Interpolation();
  ~Interpolation() {};
  void set_interpolation(ssb_common_vec::VecInterpolation v) {
    type_ = v.type;
    p_ = v.p;
  };
  void set_set_p(void (Interpolation::*set_function)(ofVec2f, int)) { set_p_ = set_function; };
  void set_type(int id) { type_ = id; };
  void set_p(ofVec2f p, int id) { return (this->*set_p_)(p, id); };
  ssb_common_vec::VecInterpolation get_interpolation() {
    return ssb_common_vec::VecInterpolation(type_, p_);
  };
  int get_type() { return type_; };
  ofVec2f get_p(int id) { return p_.at(id); };
  float F(float t) { return (this->*f_[type_])(t); };
  void Init() { return (this->*init_[type_])(); };
  void set_constant_p(ofVec2f p, int id) {};
  void set_linear_p(ofVec2f p, int id) {};
  void set_bezier_p(ofVec2f p, int id=0);
  void set_slowin_p(ofVec2f p, int id=0);
  void set_slowout_p(ofVec2f p, int id=0);
  void set_sigmoid_p(ofVec2f p, int id);
  void set_cubicbezier_p(ofVec2f p, int id);
 private:
  void (Interpolation::*init_[7])();
  float (Interpolation::*f_[7])(float);
  void (Interpolation::*set_p_)(ofVec2f, int);
  int type_;
  std::vector<ofVec2f> p_;
  void initConstant();
  void initLinear();
  void initBezier();
  void initSlowIn();
  void initSlowOut();
  void initSigmoid();
  void initCubicBezier();
  inline float Constant(float t) { return 0.0; };
  inline float Linear(float t) { return t; };
  inline float Bezier(float t) { return 2*t*(1-t)*p_.at(1).y + t*t; };
  float SlowIn(float t);
  float SlowOut(float t);
  float Sigmoid(float t);
  inline float CubicBezier(float t) {
    float s = 1 - t;
    return 3*s*s*t*p_.at(1).y + 3*t*t*s*p_.at(2).y + t*t*t;
  };
  void init(const int n);
};

} // namespace ssb_utils_interpolation

#endif
