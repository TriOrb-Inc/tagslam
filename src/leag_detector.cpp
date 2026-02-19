#include <tagslam/leag_detector.hpp>

namespace
{
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
}  // namespace

LeagDetectors::LeagDetectors(std::vector<std::string> camera_topics)
{
  const std::string str_camfile = "/params/tagslam/cameras.yaml";
  const std::string str_mkfile = "/params/markerPara_AZ.yaml";
  YAML::Node cam_conf;
  try {
    cam_conf = YAML::LoadFile(str_camfile);
  } catch (const YAML::Exception & e) {
    std::cerr << "Failed to load " << str_camfile << ": " << e.what() << std::endl;
    return;
  }

  for (const auto & cam : camera_topics) {
    leag::LentiMarkTracker lmt;
    cv::Size2i img_size;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    const bool ok = fillCameraParamsFromTagslamCameras(
      cam_conf, cam, &img_size, &camera_matrix, &dist_coeffs);
    if (!ok) {
      std::cerr << "Failed to assign camera params for topic: " << cam << std::endl;
    }

    const int res1 = lmt.setCamParams(img_size, camera_matrix, dist_coeffs);
    const int res2 = lmt.setMarkerParams(str_mkfile);
    std::cout << "cam topic: " << cam
              << " setCamParams=" << res1
              << " setMarkerParams=" << res2 << std::endl;
    LMT_list_.push_back(lmt);
  }
}

LeagDetectors::~LeagDetectors() = default;

bool LeagDetectors::detect_marker(const cv::Mat & img, ApriltagArray::SharedPtr tags, uint8_t camera_index)
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
