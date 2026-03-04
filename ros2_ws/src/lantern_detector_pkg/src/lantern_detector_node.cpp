#include "lantern_detector_pkg/lantern_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace lantern_detector_pkg {

LanternDetectorNode::LanternDetectorNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("lantern_detector", options)
{
  depth_topic_      = this->declare_parameter<std::string>("depth_topic", "/realsense/depth/image");
  semantic_topic_   = this->declare_parameter<std::string>("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
  depth_info_topic_ = this->declare_parameter<std::string>("depth_info_topic", "/realsense/depth/camera_info");

  output_topic_ = this->declare_parameter<std::string>("output_topic", "detected_lanterns");
  count_topic_  = this->declare_parameter<std::string>("count_topic", "detected_lanterns/counts");
  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "lantern_markers");

  world_frame_ = this->declare_parameter<std::string>("world_frame", "world");

  min_depth_m_ = this->declare_parameter<double>("min_depth", 0.1);
  max_depth_m_ = this->declare_parameter<double>("max_depth", 50.0);
  distance_threshold_m_ = this->declare_parameter<double>("distance_threshold", 2.0);
  min_sightings_ = this->declare_parameter<int>("min_num_sightings", 30);
  pixel_stride_ = this->declare_parameter<int>("pixel_stride", 2);
  sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 30);
  tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.2);

  marker_scale_m_ = this->declare_parameter<double>("marker_scale", 0.5);
  marker_color_ = this->declare_parameter<std::vector<double>>(
      "marker_color", std::vector<double>{1.0, 1.0, 0.0, 1.0});

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // pubs
  lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic_, 10);
  lantern_counts_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(count_topic_, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

  // camera info
  sub_depth_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_info_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lk(depth_info_mutex_);
      depth_info_ = msg;
    });

  // message_filters subs
  // message_filters subs (match publisher QoS; RELIABLE is safest with unity_ros)
  sensor_qos_ = rmw_qos_profile_default;
  sensor_qos_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  sensor_qos_.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sensor_qos_.depth = 10;

  sub_semantic_filter_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, semantic_topic_, sensor_qos_);
  sub_depth_filter_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic_, sensor_qos_);
    
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(std::max(1, sync_queue_size_)),
    *sub_semantic_filter_,
    *sub_depth_filter_);
  sync_->registerCallback(std::bind(&LanternDetectorNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "LanternDetectorNode started.");
  RCLCPP_INFO(this->get_logger(), "semantic_topic: %s", semantic_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "depth_topic   : %s", depth_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "camera_info   : %s", depth_info_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "world_frame   : %s", world_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "pixel_stride  : %d", pixel_stride_);
  RCLCPP_INFO(this->get_logger(), "sync_queue    : %d", sync_queue_size_);
  RCLCPP_INFO(this->get_logger(), "tf_timeout_s  : %.3f", tf_timeout_s_);
}

double LanternDetectorNode::get_depth_m(const cv::Mat& depth, const std::string& encoding, int u, int v) const
{
  if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) return -1.0;

  if (encoding == "16UC1") {
    const uint16_t d = depth.at<uint16_t>(v, u);
    if (d == 0) return -1.0;
    return static_cast<double>(d) * 0.001;  // mm -> m
  } else if (encoding == "32FC1") {
    const float d = depth.at<float>(v, u);
    if (!std::isfinite(d) || d <= 0.0f) return -1.0;
    return static_cast<double>(d);          // m
  }
  return -1.0;
}

void LanternDetectorNode::synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                                                const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    std::lock_guard<std::mutex> lk(depth_info_mutex_);
    info = depth_info_;
  }
  if (!info) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Waiting for depth camera info...");
    return;
  }

  cv::Mat sem_bgr;
  cv::Mat depth;
  try {
    sem_bgr = cv_bridge::toCvShare(semantic_msg, "bgr8")->image;

    if (depth_msg->encoding == "16UC1") {
      depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image;
    } else if (depth_msg->encoding == "32FC1") {
      depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "cv_bridge failed: %s", e.what());
    return;
  }

  // --- Mask: default non-zero in semantic image ---
  cv::Mat gray, mask;
  cv::cvtColor(sem_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY);

  // 轻微降噪 - 减少腐蚀以保留小目标检测（避免错过远处或快速经过的灯笼）
  // 原: erode 1, dilate 2 -> 改为: 只做轻微膨胀填补空洞
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);

  cv::Mat nonzero;
  cv::findNonZero(mask, nonzero);
  if (nonzero.empty()) return;

  const cv::Rect roi = cv::boundingRect(nonzero);
  const int stride = std::max(1, pixel_stride_);

  // Intrinsics
  const double fx = (info->p[0] != 0.0) ? info->p[0] : info->k[0];
  const double fy = (info->p[5] != 0.0) ? info->p[5] : info->k[4];
  const double cx = (info->p[2] != 0.0) ? info->p[2] : info->k[2];
  const double cy = (info->p[6] != 0.0) ? info->p[6] : info->k[5];
  if (fx <= 1e-6 || fy <= 1e-6) return;

  // Accumulate 3D points in depth camera frame (average)
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  int count = 0;

  // semantic and depth may differ in size
  const double sx = static_cast<double>(depth.cols) / static_cast<double>(mask.cols);
  const double sy = static_cast<double>(depth.rows) / static_cast<double>(mask.rows);

  for (int y = roi.y; y < roi.y + roi.height; y += stride) {
    const uchar* row = mask.ptr<uchar>(y);
    for (int x = roi.x; x < roi.x + roi.width; x += stride) {
      if (row[x] == 0) continue;

      const int u = static_cast<int>(x * sx);
      const int v = static_cast<int>(y * sy);

      const double z = get_depth_m(depth, depth_msg->encoding, u, v);
      if (!(z > min_depth_m_ && z < max_depth_m_)) continue;

      const double X = (u - cx) * z / fx;
      const double Y = (v - cy) * z / fy;

      sum_x += X; sum_y += Y; sum_z += z;
      count++;
    }
  }

  if (count <= 0) return;

  geometry_msgs::msg::PointStamped p_cam;
  p_cam.header = depth_msg->header;  // uses depth frame_id + stamp
  p_cam.point.x = sum_x / count;
  p_cam.point.y = sum_y / count;
  p_cam.point.z = sum_z / count;

  try {
    const auto p_world = tf_buffer_->transform(
      p_cam, world_frame_, tf2::durationFromSec(tf_timeout_s_));
    update_lanterns(p_world.point);
    publish_lanterns();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "TF transform failed: %s", ex.what());
  }
}

void LanternDetectorNode::update_lanterns(const geometry_msgs::msg::Point& new_position)
{
  auto it = std::find_if(detected_lanterns_.begin(), detected_lanterns_.end(),
    [&](const TrackedLantern& l){
      const double dx = l.position.x - new_position.x;
      const double dy = l.position.y - new_position.y;
      const double dz = l.position.z - new_position.z;
      return std::sqrt(dx*dx + dy*dy + dz*dz) < distance_threshold_m_;
    });

  if (it != detected_lanterns_.end()) {
    it->position.x = (it->position.x * it->count + new_position.x) / (it->count + 1);
    it->position.y = (it->position.y * it->count + new_position.y) / (it->count + 1);
    it->position.z = (it->position.z * it->count + new_position.z) / (it->count + 1);
    it->count++;
  } else {
    detected_lanterns_.push_back({new_position, 1});
  }
}

void LanternDetectorNode::publish_lanterns()
{
  const auto now = this->get_clock()->now();

  geometry_msgs::msg::PoseArray poses;
  poses.header.stamp = now;
  poses.header.frame_id = world_frame_;

  std_msgs::msg::Int32MultiArray counts;
  visualization_msgs::msg::MarkerArray markers;

  int id = 0;
  for (const auto& l : detected_lanterns_) {
    if (l.count < min_sightings_) continue;

    geometry_msgs::msg::Pose p;
    p.position = l.position;
    p.orientation.w = 1.0;

    poses.poses.push_back(p);
    counts.data.push_back(l.count);

    visualization_msgs::msg::Marker m;
    m.header = poses.header;
    m.ns = "lanterns";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = p;
    m.scale.x = m.scale.y = m.scale.z = marker_scale_m_;

    if (marker_color_.size() >= 4) {
      m.color.r = marker_color_[0];
      m.color.g = marker_color_[1];
      m.color.b = marker_color_[2];
      m.color.a = marker_color_[3];
    } else {
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
    }
    markers.markers.push_back(m);
  }

  lantern_pub_->publish(poses);
  lantern_counts_pub_->publish(counts);
  marker_pub_->publish(markers);
}

}  // namespace lantern_detector_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lantern_detector_pkg::LanternDetectorNode)