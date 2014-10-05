#ifndef SSB_COMMON_SUBSCRIBER_H_
#define SSB_COMMON_SUBSCRIBER_H_

#include "ssb_common_common.h"

namespace ssb_common_subscriber {

template<typename T> class Subscriber {
 public:
  inline T get_memory() { return memory_; };
 protected:
  ros::Subscriber subscriber_;
  T memory_;
};  
  
} // namespace ssb_common_subscriber

#endif
