// ssb packages are now dependent from Openframeworks.
// This file replaces the Openframework functions and structs used in ssb.

#ifndef SSB_COMMON_OF_H_
#define SSB_COMMON_OF_H_

#include <string>
#include <iostream>
#include <sstream>

struct ofVec2f {
  ofVec2f();
  ofVec2f(const float x_ref, const float y_ref);
  ofVec2f(const ofVec2f &obj);
  float x;
  float y;
};

float ofToFloat(const std::string& floatString);

int ofToInt(const std::string& intString);

template <class T>
std::string ofToString(const T& value) {
  std::ostringstream out;
  out << value;
  return out.str();
};

#endif
