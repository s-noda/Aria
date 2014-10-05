#include "ssb_common_of.h"

ofVec2f::ofVec2f() {
  x = 0;
  y = 0;
}

ofVec2f::ofVec2f(const float x_ref, const float y_ref) {
  x = x_ref;
  y = y_ref;
}

ofVec2f::ofVec2f(const ofVec2f& obj) {
  x = obj.x;
  y = obj.y;
}


float ofToFloat(const std::string& floatString) {
  float x = 0;
  std::istringstream cur(floatString);
  cur >> x;
  return x;
}

int ofToInt(const std::string& intString) {
  int x = 0;
  std::istringstream cur(intString);
  cur >> x;
  return x;
}
