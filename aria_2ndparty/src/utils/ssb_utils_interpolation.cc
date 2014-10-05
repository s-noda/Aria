#include "ssb_utils_interpolation.h"

namespace ssb_utils_interpolation {

Interpolation::Interpolation() : set_p_(&Interpolation::set_constant_p) {
  type_ = 0;
  init_[0] = &Interpolation::initConstant;
  init_[1] = &Interpolation::initLinear;
  init_[2] = &Interpolation::initBezier;
  init_[3] = &Interpolation::initSlowOut;
  init_[4] = &Interpolation::initSlowIn;
  init_[5] = &Interpolation::initSigmoid;
  init_[6] = &Interpolation::initCubicBezier;
  f_[0] = &Interpolation::Constant;
  f_[1] = &Interpolation::Linear;
  f_[2] = &Interpolation::Bezier;
  f_[3] = &Interpolation::SlowOut;
  f_[4] = &Interpolation::SlowIn;
  f_[5] = &Interpolation::Sigmoid;
  f_[6] = &Interpolation::CubicBezier;
}

float Interpolation::SlowOut(float t) {
  float tA = t / p_.at(1).x;
  float tB = (t - p_.at(1).x) / (1 - p_.at(1).x);
  float sB = 1 - tB;
  return t <= p_.at(1).x ?
      tA*p_.at(1).y : sB*sB*p_.at(1).y + 2*sB*tB*p_.at(2).y + tB*tB;
}

// y(t=p_.at(2).x) = p_.at(2).y is actually not true on the
// mathematical grid.
// However, when plotting p_.at(2) we assume that at t=p_.at(2).x,
// we reach p_.at(2).y.
// Therefore from the intuitive perspective, the following
// equation should work fine.
float Interpolation::SlowIn(float t) {
  float tA = t / p_.at(2).x;
  float tB = (t - p_.at(2).x) / (1 - p_.at(2).x);
  return t <= p_.at(2).x ?
      2*tA*(1-tA)*p_.at(1).y + tA*tA*p_.at(2).y : (1 - tB)*p_.at(2).y + tB;
}

// The following is not actually a sigmoid.
// Its called a sigmoid because it looks like one.
// Again, mathematically not true, but should work intuitively.
float Interpolation::Sigmoid(float t) {
  float tA = t / p_.at(2).x;
  float tB = (t - p_.at(2).x) / (p_.at(3).x - p_.at(2).x);
  float tC = (t - p_.at(3).x) / (1 - p_.at(3).x);
  float sC = 1 - tC;
  return t <= p_.at(2).x ?
      2*tA*(1-tA)*p_.at(1).y + tA*tA*p_.at(2).y :
      (t <= p_.at(3).x ?
       (1 - tB)*p_.at(2).y + tB*p_.at(3).y :
       sC*sC*p_.at(3).y + 2*sC*tC*p_.at(4).y + tC*tC);
}

void Interpolation::init(const int n) {
  p_.clear();
  p_.resize(n);
  p_.at(0) = ofVec2f(0.0, 0.0);
  p_.at(n-1) = ofVec2f(1.0, 1.0);
}

void Interpolation::initConstant() {
  type_ = 0;
  p_.clear();
  p_.resize(2);
  p_.at(0) = ofVec2f(0.0, 0.0);
  p_.at(1) = ofVec2f(1.0, 0.0);
}

void Interpolation::initLinear() {
  type_ = 1;
  init(2);
}

void Interpolation::initBezier() {
  type_ = 2;
  init(3);
  p_.at(1) = ofVec2f(0.0, 1.0);
}

void Interpolation::initSlowOut() {
  type_ = 3;
  init(4);
  ofVec2f p1(0.5, 0.8); // p1.x < p1.y  
  p_.at(1) = p1;
  p_.at(2) = ofVec2f(p1.x/p1.y, 1.0);
}

void Interpolation::initSlowIn() {
  type_ = 4;
  init(4);
  ofVec2f p2(0.5, 0.2); // p2.x > p2.y
  p_.at(1) = ofVec2f((p2.x-p2.y)/(1-p2.y), 0.0);
  p_.at(2) = p2;
}

void Interpolation::initSigmoid() {
  type_ = 5;
  init(6);
  ofVec2f p2(0.3, 0.2); // p2.x < p3.x, p2.y < p3.y
  ofVec2f p3(0.7, 0.8);
  p_.at(1) = ofVec2f((p2.x*p3.y-p3.x*p2.y)/(p3.y-p2.y), 0.0);
  p_.at(2) = p2;
  p_.at(3) = p3;
  p_.at(4) = ofVec2f((p3.x*(1-p2.y)-p2.x*(1-p3.y))/(p3.y-p2.y), 1.0);
}

void Interpolation::initCubicBezier() {
  type_ = 6;
  init(4);
  p_.at(1) = ofVec2f(0.0, 0.5);
  p_.at(2) = ofVec2f(1.0, 0.5);
}

void Interpolation::set_bezier_p(ofVec2f p, int id) {
  p_.at(1) = p; 
}

void Interpolation::set_slowout_p(ofVec2f p, int id) {
  p_.at(1) = p;
  if (p_.at(1).x > p_.at(1).y)
    p_.at(1).x = p_.at(1).y;
  p_.at(2) = ofVec2f(p_.at(1).x/p_.at(1).y, 1.0);
}

void Interpolation::set_slowin_p(ofVec2f p, int id) {
  p_.at(2) = p;
  if (p_.at(2).x < p_.at(2).y)
    p_.at(2).x = p_.at(2).y;
  p_.at(1) = ofVec2f((p_.at(2).x-p_.at(2).y)/(1-p_.at(2).y), 0.0);
}

void Interpolation::set_sigmoid_p(ofVec2f p, int id) {
  if (id == 2) {
    p_.at(2) = p;
    if (p_.at(2).x > p_.at(3).x)
      p_.at(2).x = p_.at(3).x;
    if (p_.at(2).y > p_.at(3).y)
      p_.at(2).y = p_.at(3).y;
  } else if (id == 3) {
    p_.at(3) = p;
    if (p_.at(3).x < p_.at(2).x)
      p_.at(3).x = p_.at(2).x;
    if (p_.at(3).y < p_.at(2).y)
      p_.at(3).y = p_.at(2).y;
  } else {
    return;
  }  
  p_.at(1) = ofVec2f(
      (p_.at(2).x*p_.at(3).y-p_.at(3).x*p_.at(2).y)/(p_.at(3).y-p_.at(2).y),
      0.0);
  p_.at(4) = ofVec2f(
      (p_.at(3).x*(1-p_.at(2).y)-p_.at(2).x*(1-p_.at(3).y))/(p_.at(3).y-p_.at(2).y),
      1.0);
}

void Interpolation::set_cubicbezier_p(ofVec2f p, int id) {
  if (id == 1) {
    p_.at(1) = p;
    if (p_.at(1).x > p_.at(2).x)
      p_.at(1).x = p_.at(2).x;
  } else if (id == 2) {
    p_.at(2) = p;
    if (p_.at(2).x < p_.at(1).x)
      p_.at(2).x = p_.at(1).x;
  }
}

} // namespace ssb_utils_interpolation
