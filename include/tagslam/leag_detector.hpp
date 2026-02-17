#ifndef TAGSLAM__LEAG_DETECTOR_HPP_
#define TAGSLAM__LEAG_DETECTOR_HPP__

#include "LentiMarkTracker.h"

#include <chrono>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cassert>

#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

using ApriltagArray = apriltag_msgs::msg::AprilTagDetectionArray;
class LeagDetectors
{
public:
    LeagDetectors(std::vector<std::string> camera_topics);
    ~LeagDetectors();
    bool detect_marker(const cv::Mat &img, ApriltagArray::SharedPtr tags, uint8_t camera_index);

private:
    std::vector<leag::LentiMarkTracker> LMT_list_;

};

#endif
