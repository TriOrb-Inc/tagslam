#pragma once
#include <opencv2/core.hpp>   // cv::Mat だけ欲しいなら core で十分
#include <memory>

#include <yaml-cpp/yaml.h>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

namespace tagslam::leagwrap {

using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;

// PImpl：実体を隠す
class Tracker {
public:
  Tracker();
  ~Tracker();

  Tracker(const Tracker&) = delete;
  Tracker& operator=(const Tracker&) = delete;

  void initializeLeagDetectors(std::vector<std::string> camera_topics, std::string str_camfile);
  bool detect_marker(const cv::Mat &img, ApriltagArray::SharedPtr tags, size_t camera_index);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace tagslam::leagwrap