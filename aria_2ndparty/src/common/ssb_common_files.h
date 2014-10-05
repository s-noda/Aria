#ifndef SSB_COMMON_FILES_H_
#define SSB_COMMON_FILES_H_

#include "ssb_common_common.h"
#include "ssb_srvs_msgs/MultiString.h"

namespace ssb_common_files {

class Files {
 public:
  Files(ros::NodeHandle &nh) {
    file_client_ = nh.serviceClient<ssb_srvs_msgs::MultiString>("/py_file");
    wav_playtime_client_ = nh.serviceClient<ssb_srvs_msgs::MultiString>("/py_wav");
  }
  ~Files() {};
  std::vector<std::string> getList(std::string directory, std::string filetype) {
    ssb_srvs_msgs::MultiString srv;
    srv.request.data.push_back(directory+", "+filetype);
    if (file_client_.call(srv)) {
      return srv.response.data;
    }
    return std::vector<std::string>(0);
  };
  std::vector<float> getWavPlayTime(std::vector<std::string> wav_file_names) {
    ssb_srvs_msgs::MultiString srv;
    srv.request.data = wav_file_names;
    if (wav_playtime_client_.call(srv)) {
      std::vector<float> result(srv.response.data.size());
      for (int i = 0; i < result.size(); ++i)
        result[i] = ofToFloat(srv.response.data[i].c_str());
      return result;
    }
    return std::vector<float>(0);
  };
 private:
  ros::ServiceClient file_client_;
  ros::ServiceClient wav_playtime_client_;
};

} // namespace ssb_common_files

#endif
