#include <map>
#include <fstream>
#include <sstream>
#include <time.h>
#include "../../aria_2ndparty/src/common/ssb_common_common.h"
#include "../../aria_2ndparty/src/sensors/ssb_sensors_sound.h"
#include "std_msgs/String.h"

void ssb_sensors_sound::Sound::Loop() {
}

class Sound : public ssb_sensors_sound::Sound {
public:
  explicit Sound(ros::NodeHandle &nh) : ssb_sensors_sound::Sound(nh) {
    publisher_ = nh.advertise<std_msgs::String>("/aria/commandline", 100);
    random_count_ = 0;
    behaviour_cycle_ = 600;
    std::string filename;
    if (nh.getParam("/v2p/filename", filename))
      Load(filename);
  };
  virtual ~Sound() {};
  void Load(std::string filename) {
    std::ifstream file(filename.c_str());
    while ( !file.eof() ) {
      std::string text;
      file >> text;
      std::string val;
      std::string::size_type n = text.find(":");
      if (n == std::string::npos)
	continue;
      else
	val = text.substr(0,n);
      int head=n+1;
      while (1) {
	std::string::size_type n1;
	n1 = text.find(",",head);
	if (n1 == std::string::npos) {
	  voice_to_pose_[text.substr(head, (n1-head))] = val;
	  break;
	} else {
	  voice_to_pose_[text.substr(head, (n1-head))] = val;
	  head = n1+1;
	}
      }
    }
    file.close();
    for (std::map<std::string, std::string>::const_iterator it = voice_to_pose_.begin();
	 it != voice_to_pose_.end(); ++it)
      std::cout << it->first << " " << it->second << std::endl;
    srand((unsigned int)time(NULL));
    ROS_INFO("load done");
  };
  void Loop() {
    if (memory_.texts.size() > 0) {
      std::cout << memory_.texts[0] << std::endl;
      // break down sentence
      int mark_type = 0;
      std::vector<int> mark_eng;
      std::vector<int> mark_jpn_kanji;
      std::vector<int> mark_jpn_hira;
      std::vector<std::string> words;
      for (int i = 0; i < memory_.texts[0].size();) {
	if ((int)memory_.texts[0].c_str()[i] > 0) { // if english
	  if (mark_type != 1) {
	    mark_eng.push_back(i*100+1);
	    mark_type = 1;
	  } else {
	    mark_eng.at(mark_eng.size()-1) += 1;
	  }
	  i++;
	} else if ((int)memory_.texts[0].c_str()[i] == -29) { // if not kanji
	  if (mark_type != 3) {
	    mark_jpn_hira.push_back(i*100+3);
	    mark_type = 3;
	  } else {
	    mark_jpn_hira.at(mark_jpn_hira.size()-1) += 3;
	  }
	  i += 3;
	} else { // if kanji
	  if (mark_type != 2) {
	    mark_jpn_kanji.push_back(i*100+3);
	    mark_type = 2;
	  } else {
	    mark_jpn_kanji.at(mark_jpn_kanji.size()-1) += 3;
	  }
	  i += 3;
	}
      }
      words.resize(mark_eng.size() + mark_jpn_kanji.size() + mark_jpn_hira.size());
      for (int i = 0; i < mark_eng.size(); ++i)
	words[i] = memory_.texts[0].substr(mark_eng.at(i)/100, mark_eng.at(i)%100);
      for (int i = 0; i < mark_jpn_kanji.size(); ++i)
	words[i+mark_eng.size()]
	  = memory_.texts[0].substr(mark_jpn_kanji.at(i)/100, mark_jpn_kanji.at(i)%100);
      for (int i = 0; i < mark_jpn_hira.size(); ++i)
	words[i+mark_eng.size()+mark_jpn_kanji.size()]
	  = memory_.texts[0].substr(mark_jpn_hira.at(i)/100, mark_jpn_hira.at(i)%100);
      // find pose keyword
      for (int i = 0; i < words.size(); ++i) {
	if (voice_to_pose_.find(words.at(i)) == voice_to_pose_.end()) {
	} else {
	  std_msgs::String msg;
	  msg.data = voice_to_pose_[words.at(i)];
	  publisher_.publish(msg);
	  random_count_ = 0;
	}
      }
      if (random_count_ > (behaviour_cycle_ - rand()%400)) {
	std::stringstream ss;
	ss << rand()%40;
	std::string random = ss.str();
	if (voice_to_pose_.find(random) == voice_to_pose_.end()) {
	} else {
	  std_msgs::String msg;
	  msg.data = voice_to_pose_[random];
	  publisher_.publish(msg);
	}
	random_count_ = 0;
      }
      memory_.texts.clear();
    } else {
      if (random_count_ > (behaviour_cycle_ - rand()%200)) {
	std::stringstream ss;
	ss << rand()%20;
	std::string random = ss.str();
	if (voice_to_pose_.find(random) == voice_to_pose_.end()) {
	} else {
	  std_msgs::String msg;
	  msg.data = voice_to_pose_[random];
	  publisher_.publish(msg);
	}
	random_count_ = 0;
      }
    }
    random_count_++;
  };
private:
  std::map<std::string, std::string> voice_to_pose_;
  ros::Publisher publisher_;
  int random_count_;
  int behaviour_cycle_;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "voice_to_pose");
  ros::NodeHandle nh;
  Sound sound(nh);
  ros::Rate loop(30);
  while(ros::ok()) {
    ros::spinOnce();
    sound.Loop();
    loop.sleep();
  }
}
