// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#ifdef USE_CV_BRIDGE_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <tagslam/logging.hpp>
#include <tagslam/sync_and_detect.hpp>

namespace tagslam
{
using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

std::string normalizeTopic(const std::string & topic)
{
  if (topic.empty()) {
    return topic;
  }
  if (topic.front() == '/') {
    return topic.substr(1);
  }
  return topic;
}

bool fillCameraParamsFromTagslamCameras(
  const YAML::Node & root, const std::string & camera_topic,
  cv::Size2i * img_size, cv::Mat * camera_matrix, cv::Mat * dist_coeffs)
{
  if (!root || !root.IsMap()) {
    std::cerr << "cameras.yaml root is not a map." << std::endl;
    return false;
  }

  const std::string normalized_topic = normalizeTopic(camera_topic);
  for (const auto & entry : root) {
    const auto camera = entry.second;
    if (!camera["image_topic"] || !camera["intrinsics"] ||
      !camera["distortion_coeffs"] || !camera["resolution"])
    {
      continue;
    }

    const std::string yaml_topic =
      normalizeTopic(camera["image_topic"].as<std::string>());
    if (yaml_topic != normalized_topic) {
      continue;
    }

    const auto intrinsics = camera["intrinsics"];
    const auto distortion = camera["distortion_coeffs"];
    const auto resolution = camera["resolution"];
    if (!intrinsics.IsSequence() || intrinsics.size() < 4 ||
      !distortion.IsSequence() || distortion.size() < 4 ||
      !resolution.IsSequence() || resolution.size() < 2)
    {
      std::cerr << "Invalid camera calibration format for topic: "
                << camera_topic << std::endl;
      return false;
    }

    const double fx = intrinsics[0].as<double>();
    const double fy = intrinsics[1].as<double>();
    const double cx = intrinsics[2].as<double>();
    const double cy = intrinsics[3].as<double>();

    const double k1 = distortion[0].as<double>();
    const double k2 = distortion[1].as<double>();
    const double k3 = distortion[2].as<double>();
    const double k4 = distortion[3].as<double>();

    const int cols = resolution[0].as<int>();
    const int rows = resolution[1].as<int>();

    *img_size = cv::Size2i(cols, rows);
    *camera_matrix = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);
    *dist_coeffs = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);
    return true;
  }

  std::cerr << "No camera entry found in cameras.yaml for topic: "
            << camera_topic << std::endl;
  return false;
}

void reverseCornerOrderInPlace(std::vector<std::vector<cv::Point2f>> & corners)
{
  for (auto & corner : corners) {
    assert(corner.size() == 4);
    std::reverse(corner.begin(), corner.end());
  }
}

Publisher::Publisher(
  rclcpp::Node * node, const svec & tag_topics, const svec & odom_topics)
{
  for (const auto & tt : tag_topics) {
    tag_pub_.push_back(node->create_publisher<ApriltagArray>(tt, 10));
  }
  for (const auto & ot : odom_topics) {
    odom_pub_.push_back(node->create_publisher<Odometry>(ot, 10));
  }
}

void Publisher::callbackTags(const VecApriltagArrayPtr & tagMsgs)
{
  publishTags(tagMsgs);
}

void Publisher::callbackTagsAndOdom(
  const VecApriltagArrayPtr & tagMsgs, const VecOdometryPtr & odoms)
{
  publishTags(tagMsgs);
  for (size_t i = 0; i < odoms.size(); i++) {
    odom_pub_[i]->publish(*odoms[i]);
  }
}

void Publisher::publishTags(const VecApriltagArrayPtr & tagMsgs)
{
  for (size_t i = 0; i < tagMsgs.size(); i++) {
    tag_pub_[i]->publish(*tagMsgs[i]);
  }
}

SyncAndDetect::SyncAndDetect(const rclcpp::NodeOptions & opt)
: Node("sync_and_detect", opt),
  detector_loader_("apriltag_detector", "apriltag_detector::Detector")
{
  getTopics();
  if (declare_parameter<bool>("publish", true)) {
    pub_ = std::make_shared<Publisher>(this, tag_topics_, synced_odom_topics_);
    listener_ = pub_;
  }
  subscribe(image_topics_, odom_topics_, detector_names_);

#ifdef USE_LEAG_DETECTOR
  puts("using LEAG detector");
  detect_leags_ = declare_parameter<bool>("detect_leags", false);
  svec leag_camera_topics;
  leag_camera_topics.reserve(image_topics_.size());
  for (const auto & image_topic : image_topics_) {
    leag_camera_topics.push_back(image_topic.first);
  }
  // initializeLeagDetectors(leag_camera_topics);
  puts("initialized LEAG detectors");
#endif

}

SyncAndDetect::~SyncAndDetect()
{
  image_exact_sync_.reset();
  image_approx_sync_.reset();
  image_odom_exact_sync_.reset();
  image_odom_approx_sync_.reset();
  detectors_.clear();
  pub_.reset();
  for (const auto & type : detector_types_) {
    detector_loader_.unloadLibraryForClass(
      "apriltag_detector_" + type + "::Detector");
  }
}

void SyncAndDetect::setListener(
  const std::shared_ptr<SyncAndDetectListener> & m)
{
  listener_ = m;
}

void SyncAndDetect::getTopics()
{
  parseCameras(loadFileFromParameter("cameras"));
  auto conf = loadFileFromParameter("tagslam_config");
  if (conf["bodies"]) {
    parseBodies(conf["bodies"]);
  }
}

void SyncAndDetect::parseCameras(const YAML::Node & conf)
{
  svec cam_names;
  for (const auto & c : conf) {
    const auto name = c.first.as<string>();
    if (name.find("cam") != string::npos) {
      cam_names.push_back(name);
    }
  }
  std::sort(cam_names.begin(), cam_names.end(), [](string a, string b) {
    return a < b;
  });
  for (const auto & cam : cam_names) {
    const auto & c = conf[cam];
    if (!c["image_topic"] || !c["tag_topic"]) {
      BOMB_OUT("must specify image_topic and tag_topic for camera " << cam);
    }
    tag_topics_.push_back(c["tag_topic"].as<string>());
    image_topics_.push_back(
      {c["image_topic"].as<string>(),
       c["image_transport"] ? c["image_transport"].as<string>() : "raw"});
    detector_names_.push_back(
      c["tag_detector"] ? c["tag_detector"].as<string>() : "umich");
    LOG_INFO(
      "camera " << cam << " topic: " << image_topics_.back().first
                << " transp: " << image_topics_.back().second
                << " detect: " << detector_names_.back());
  }
}

void SyncAndDetect::parseBodies(const YAML::Node & conf)
{
  for (const auto & body : conf) {
    if (body.IsMap()) {
      for (const auto & bm : body) {
        auto node = bm.second;
        if (node["odom_topic"]) {
          const auto topic = node["odom_topic"].as<std::string>();
          odom_topics_.push_back(topic);
          synced_odom_topics_.push_back(topic + "_synced");
          LOG_INFO(
            "body " << bm.first.as<std::string>()
                    << " has odom topic: " << topic);
        }
      }
    }
  }
}

YAML::Node SyncAndDetect::loadFileFromParameter(const string & name)
{
  const string p = declare_parameter<string>(name, "");
  if (p.empty()) {
    BOMB_OUT("parameter " << name << " must be specified and non-empty!");
  }
  YAML::Node config = YAML::LoadFile(p);
  if (config.IsNull()) {
    BOMB_OUT("cannot open config file: " << p);
  }
  return (config);
}

void SyncAndDetect::callbackImageAndOdom(
  const VecImagePtr & imgs, const VecOdometryPtr & odoms)
{
  VecApriltagArrayPtr tagMsgs;
  num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
  num_frames_++;
  if (num_frames_ % 10 == 0) {
    LOG_INFO("frame " << num_frames_ << " total tags: " << num_tags_detected_);
  }
  if (listener_) {
    listener_->callbackTagsAndOdom(tagMsgs, odoms);
  }
}

void SyncAndDetect::callbackImage(const VecImagePtr & imgs)
{
  VecApriltagArrayPtr tagMsgs;
  num_tags_detected_ += tagsFromImages(imgs, &tagMsgs);
  num_frames_++;
  if (num_frames_ % 10 == 0) {
    LOG_INFO("frame " << num_frames_ << " total tags: " << num_tags_detected_);
  }
  if (listener_) {
    listener_->callbackTags(tagMsgs);
  }
}

size_t SyncAndDetect::getNumberOfTagsDetected() const
{
  return (num_tags_detected_);
}
void SyncAndDetect::subscribe(
  const std::vector<std::pair<std::string, std::string>> & img_topics,
  const svec & odom_topics, const svec & detectors)
{
  svec imgs;
  for (const auto & img : img_topics) {
    imgs.push_back(img.first);
  }
  assert(img_topics.size() == detectors.size());

  for (size_t i = 0; i < img_topics.size(); i++) {
    const auto & topic = img_topics[i].first;
    declare_parameter<string>(topic + ".image_transport", img_topics[i].second);
    detectors_.push_back(detector_loader_.createSharedInstance(
      "apriltag_detector_" + detectors[i] + "::Detector"));
    detector_types_.insert(detectors[i]);
  }
  const bool use_approx_sync =
    declare_parameter<bool>("use_approximate_sync", true);
  LOG_INFO("using approximate sync: " << (use_approx_sync ? "YES" : "NO"));
  const int qs = 10;  // queue size
  if (odom_topics.empty()) {
    if (use_approx_sync) {
      image_approx_sync_ = std::make_shared<ImageApproxSync>(
        this, svecvec({imgs}),
        std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
    } else {
      image_exact_sync_ = std::make_shared<ImageExactSync>(
        this, svecvec({imgs}),
        std::bind(&SyncAndDetect::callbackImage, this, _1), qs);
    }
  } else {
    if (use_approx_sync) {
      image_odom_approx_sync_ = std::make_shared<ImageAndOdomApproxSync>(
        this, svecvec({imgs, odom_topics}),
        std::bind(&SyncAndDetect::callbackImageAndOdom, this, _1, _2), qs);
    } else {
      image_odom_exact_sync_ = std::make_shared<ImageAndOdomExactSync>(
        this, svecvec({imgs, odom_topics}),
        std::bind(&SyncAndDetect::callbackImageAndOdom, this, _1, _2), qs);
    }
  }
}

size_t SyncAndDetect::tagsFromImages(
  const VecImagePtr & imgs, VecApriltagArrayPtr * tagMsgs)
{
  assert(detectors_.size() == imgs.size());
  size_t num_tags{0};
  for (size_t i = 0; i < imgs.size(); i++) {
    const auto & img = imgs[i];
    auto tags = std::make_shared<ApriltagArray>();
    tagMsgs->push_back(tags);
    tags->header = img->header;
    cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvShare(img, "mono8");
    if (!cvImg) {
      BOMB_OUT("cannot convert image to mono!");
    }
    detectors_[i]->detect(cvImg->image, tags.get());
    auto detected_tags = tags->detections.size();

#ifdef USE_LEAG_DETECTOR
    if(detect_leags_ && detected_tags > 0){ 
        if( !detect_marker(cvImg->image, tags, i) ){
          tags->detections.clear();
        }
        detected_tags = tags->detections.size();
    }
#endif
    num_tags += detected_tags;
  }
  return (num_tags);
}




#ifdef USE_LEAG_DETECTOR
void SyncAndDetect::initializeLeagDetectors(std::vector<std::string> camera_topics)
{     
  const std::string str_camfile = get_parameter("cameras").as_string();
  const std::string str_mkfile = "/params/markerPara_AZ.yaml";
  YAML::Node cam_conf;
  try {
    cam_conf = YAML::LoadFile(str_camfile);
  } catch (const YAML::Exception & e) {
    std::cerr << "Failed to load " << str_camfile << ": " << e.what() << std::endl;
    return;
  }

  for (const auto & cam : camera_topics) {
    std::cout << "LEAG detector for camera topic: " << cam << std::endl;
  }
  LMT_list_.reserve(camera_topics.size());
  for (const auto & cam : camera_topics) {
    leag::LentiMarkTracker lmt;
    cv::Size2i img_size;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    const bool ok = fillCameraParamsFromTagslamCameras( cam_conf, cam, &img_size, &camera_matrix, &dist_coeffs);
    if (!ok) {
      std::cerr << "Failed to assign camera params for topic: " << cam << std::endl;
      continue;
    }

    const int res1 = lmt.setCamParams(img_size, camera_matrix, dist_coeffs);
    const int res2 = lmt.setMarkerParams("/params/markerPara_AZ.yaml");
    std::cout << "cam topic: " << cam << " setCamParams=" << res1 << " setMarkerParams=" << res2 << std::endl;
    LMT_list_.push_back(lmt);
  }
}


bool SyncAndDetect::detect_marker(const cv::Mat & img, ApriltagArray::SharedPtr tags, uint8_t camera_index)
{
  if (camera_index >= LMT_list_.size()) {
    std::cerr << "Invalid camera index for LEAG detection: " << camera_index << std::endl;
    return false;
  }
  std::vector<std::vector<cv::Point2f>> tagslam_corners;
  std::vector<int> ids;
  for (const auto & det : tags->detections) {
    std::vector<cv::Point2f> corner(4);
    for (size_t i = 0; i < 4; i++) {
      corner[i] = cv::Point2f(det.corners[i].x, det.corners[i].y);
    }
    tagslam_corners.push_back(corner);
    ids.push_back(det.id);
  }
  reverseCornerOrderInPlace(tagslam_corners);
  int res;
  if ((res = LMT_list_[camera_index].detect_nonAR(img, ids, tagslam_corners)) < 0) {
    std::cerr << "LEAG detect error for camera index " << camera_index << ": " << res << std::endl;
    return false;
  }

  std::vector<leag::LentiMarkTracker::ResultData> m_data;
  int candidateNum = LMT_list_[camera_index].getResult(m_data);
  if (candidateNum < 0) {
    return false;
  }

  std::vector<int> ids_cor;
  std::vector<int> ids_cen;
  std::vector<std::vector<cv::Point2f>> leag_corners;
  std::vector<cv::Point2f> centers;
  candidateNum = LMT_list_[camera_index].getResultPKGData(ids_cor, leag_corners);
  candidateNum = LMT_list_[camera_index].getResultPKGCenterData(ids_cen, centers);
  if (candidateNum < 0) {
    return false;
  }
  if (ids_cen.size() != ids_cor.size() || ids_cen.size() != leag_corners.size() ||
    ids_cen.size() != centers.size())
  {
    std::cerr << "LEAG result size mismatch: ids_cen=" << ids_cen.size()
              << " ids_cor=" << ids_cor.size()
              << " corners=" << leag_corners.size()
              << " centers=" << centers.size() << std::endl;
    return false;
  }

  // Convert LEAG corner order back to TagSLAM corner order before publishing.
  reverseCornerOrderInPlace(leag_corners);

  tags->detections.clear();
  tags->detections.reserve(ids_cen.size());
  for (size_t i = 0; i < ids_cen.size(); ++i) {
    apriltag_msgs::msg::AprilTagDetection det;
    det.family = "leag";
    det.hamming = 0;
    det.goodness = 0.0;
    det.decision_margin = 0.0;
    det.id = ids_cen[i];
    det.centre.x = centers[i].x;
    det.centre.y = centers[i].y;
    for (size_t j = 0; j < 4; ++j) {
      det.corners[j].x = leag_corners[i][j].x;
      det.corners[j].y = leag_corners[i][j].y;
    }
    tags->detections.push_back(det);
  }
  return true;
}
#endif

}  // namespace tagslam
