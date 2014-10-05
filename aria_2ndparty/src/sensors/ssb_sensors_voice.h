#ifndef SSB_SENSORS_EVENT_H_
#define SSB_SENSORS_EVENT_H_

#include "ssb_common_common.h"
#include "std_msgs/String.h"

namespace ssb_media_event {

class Event {
 public:
  explicit Event(ros::NodeHandle &nh) {};
  virtual ~Event() {};
  void publish(ssb_common_vec::VecEvent vec) {
    std_msgs::String event_msg;
    event_msg.data = vec.msg + ", "
        + ofToString(vec.start_time) + "; "
        + ofToString(vec.end_time == -1 ? static_cast<int>(vec.end_time) : vec.end_time) + ", "
        + vec.args;
    publisher_.publish(event_msg);
  };
 protected:
  ros::Publisher publisher_;
};


class Voice : public Event {
 public:
  explicit Voice(ros::NodeHandle &nh) : Event(nh) {
    publisher_ = nh.advertise<std_msgs::String>("/py_voice", 100);
  };
  ~Voice() {};
  void publish(ssb_common_vec::VecVoice vec) {
    publish_voice(vec.msg, vec.start_time, vec.end_time, ofToFloat(vec.args));
  };
  void publish_voice(std::string filename, float start=0.0, float end=-1,
               float amp=1.0, float speed=1.0) {
    std_msgs::String speech;
    speech.data = filename + ", "
        + ofToString(start) + ", "
        + ofToString(end == -1 ? static_cast<int>(end) : end) + ", "
        + ofToString(amp) + ", "
        + ofToString(speed);
    publisher_.publish(speech);
  };
  void publish_quiet() {
    std_msgs::String speech;
    speech.data = "quit";
    publisher_.publish(speech);
  };
  void publish_continue(int target=0, float end=-1, float amp=1.0, float speed=1.0) {
    std_msgs::String speech;
    speech.data = "continue, "
        + ofToString(target) + ", "
        + ofToString(end == -1 ? static_cast<int>(end) : end) + ", "
        + ofToString(amp) + ", "
        + ofToString(speed);
    publisher_.publish(speech);
  };
  void publish_interrupt(std::string filename, float start=0.0, float end=-1,
             float amp=1.0, float speed=1.0) {
    std_msgs::String speech;
    speech.data = "interrupt:" + filename + ", "
        + ofToString(start) + ", "
        + ofToString(end == -1 ? static_cast<int>(end) : end) + ", "
        + ofToString(amp) + ", "
        + ofToString(speed);
    publisher_.publish(speech);
  };
};

} // namespace ssb_media_event

#endif
