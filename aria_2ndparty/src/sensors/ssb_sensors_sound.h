#ifndef SSB_SENSORS_SOUND_H_
#define SSB_SENSORS_SOUND_H_

#include "ssb_common_common.h"
#include "jsk_gui_msgs/VoiceMessage.h"

namespace ssb_sensors_sound {

class Sound : public ssb_common_subscriber::Subscriber<jsk_gui_msgs::VoiceMessage> {
 public:
  explicit Sound(ros::NodeHandle &nh) {
    subscriber_ = nh.subscribe("/Tablet/voice", 100, &Sound::CallBack, this);
  };
  virtual ~Sound() {};
  void Loop();
 private:
  void CallBack(const jsk_gui_msgs::VoiceMessage &msg) { memory_ = msg; };
};

} // namespace ssb_sensors_sound

#endif
