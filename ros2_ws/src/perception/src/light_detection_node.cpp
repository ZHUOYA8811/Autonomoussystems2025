#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <optional>

class LightDetectionNode : public rclcpp::Node {
public:
  LightDetectionNode()
  : Node("light_detection_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---------- Parameters ----------
    semantic_topic_ = this->declare_parameter<std::string>("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
    depth_topic_    = this->declare_parameter<std::string>("depth_topic",    "/realsense/depth/image");
    cam_info_topic_ = this->declare_parameter<std::string>("camera_info_topic", "/realsense/depth/camera_info");

    world_frame_    = this->declare_parameter<std::string>("world_frame", "world");

    // Your lantern color (OpenCV uses BGR if we convert to bgr8)
    // You said lantern seems to be (4, 235, 255). Treat as BGR.
    target_b_ = this->declare_parameter<int>("target_b", 4);
    target_g_ = this->declare_parameter<int>("target_g", 235);
    target_r_ = this->declare_parameter<int>("target_r", 255);

    tol_b_ = this->declare_parameter<int>("tol_b", 10);
    tol_g_ = this->declare_parameter<int>("tol_g", 20);
    tol_r_ = this->declare_parameter<int>("tol_r", 20);

    min_area_ = this->declare_parameter<int>("min_area", 80);     // reject tiny blobs
    depth_window_ = this->declare_parameter<int>("depth_window", 2); // radius: 2 => 5x5 median

    publish_debug_mask_ = this->declare_parameter<bool>("publish_debug_mask", false);

    // ---------- Publishers ----------
    detected_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
    marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("/lantern_marker", 10);

    if (publish_debug_mask_) {
      mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lantern_mask", 10);
    }

    // ---------- Subscriptions ----------
    // Use sensor QoS for images
    auto qos = rclcpp::SensorDataQoS();

    sem_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      semantic_topic_, qos,
      std::bind(&LightDetectionNode::onSemantic, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, qos,
      std::bind(&LightDetectionNode::onDepth, this, std::placeholders::_1));

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      cam_info_topic_, qos,
      std::bind(&LightDetectionNode::onCamInfo, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "light_detection_node started.");
    RCLCPP_INFO(this->get_logger(), "semantic_topic: %s", semantic_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "depth_topic   : %s", depth_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_info   : %s", cam_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "world_frame   : %s", world_frame_.c_str());
  }

private:
  void onCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cam_info_ = *msg;
    cam_info_frame_ = msg->header.frame_id;
  }

  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Cache the latest depth image + metadata
    std::lock_guard<std::mutex> lock(mutex_);
    last_depth_msg_ = msg;
    try {
      // Do NOT force encoding here; keep original
      last_depth_cv_ = cv_bridge::toCvShare(msg)->image.clone();
      last_depth_encoding_ = msg->encoding;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Depth cv_bridge failed: %s", e.what());
      last_depth_cv_.reset();
    }
  }

  void onSemantic(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Need depth + camera_info before we can output 3D
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    cv::Mat depth_cv;
    std::string depth_encoding;
    sensor_msgs::msg::CameraInfo cam_info;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!last_depth_msg_ || !last_depth_cv_.has_value() || !cam_info_.has_value()) {
        // Not ready yet
        return;
      }
      depth_msg = last_depth_msg_;
      depth_cv = last_depth_cv_.value().clone();
      depth_encoding = last_depth_encoding_;
      cam_info = cam_info_.value();
    }

    // Convert semantic image to BGR8 (OpenCV-friendly)
    cv::Mat sem_bgr;
    try {
      sem_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Semantic cv_bridge failed: %s", e.what());
      return;
    }

    // Build mask via inRange around (B,G,R) = (4,235,255) with tolerance
    auto clamp8 = [](int v) { return std::max(0, std::min(255, v)); };

    cv::Scalar lower(
      clamp8(target_b_ - tol_b_),
      clamp8(target_g_ - tol_g_),
      clamp8(target_r_ - tol_r_)
    );
    cv::Scalar upper(
      clamp8(target_b_ + tol_b_),
      clamp8(target_g_ + tol_g_),
      clamp8(target_r_ + tol_r_)
    );

    cv::Mat mask;
    cv::inRange(sem_bgr, lower, upper, mask);

    // Clean noise a bit
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    if (publish_debug_mask_ && mask_pub_) {
      auto out = cv_bridge::CvImage(msg->header, "mono8", mask).toImageMsg();
      mask_pub_->publish(*out);
    }

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
      return;
    }

    // Pick the largest blob
    int best_idx = -1;
    double best_area = 0.0;
    for (int i = 0; i < (int)contours.size(); i++) {
      double a = cv::contourArea(contours[i]);
      if (a > best_area) {
        best_area = a;
        best_idx = i;
      }
    }

    if (best_idx < 0 || best_area < (double)min_area_) {
      return;
    }

    // Centroid
    cv::Moments m = cv::moments(contours[best_idx]);
    if (m.m00 <= 1e-6) return;

    int u = (int)(m.m10 / m.m00);
    int v = (int)(m.m01 / m.m00);

    // Bounds check with depth image size
    if (u < 0 || v < 0 || u >= depth_cv.cols || v >= depth_cv.rows) {
      return;
    }

    // Depth at (u,v): use median in a small window for stability
    double z_m = getDepthMedian(depth_cv, depth_encoding, u, v, depth_window_);
    if (!(z_m > 0.05 && z_m < 50.0)) { // reject invalid / ridiculous depth
      return;
    }

    // Camera intrinsics from CameraInfo (K)
    const double fx = cam_info.k[0];
    const double fy = cam_info.k[4];
    const double cx = cam_info.k[2];
    const double cy = cam_info.k[5];

    if (fx <= 1e-6 || fy <= 1e-6) return;

    // Back-project to camera frame (depth camera optical frame)
    const double x = (u - cx) * z_m / fx;
    const double y = (v - cy) * z_m / fy;

    geometry_msgs::msg::PointStamped p_cam;
    p_cam.header.stamp = msg->header.stamp;          // use semantic timestamp
    p_cam.header.frame_id = depth_msg->header.frame_id.empty()
                              ? cam_info.header.frame_id
                              : depth_msg->header.frame_id; // prefer depth frame
    p_cam.point.x = x;
    p_cam.point.y = y;
    p_cam.point.z = z_m;

    // Transform to world_frame_
    geometry_msgs::msg::PointStamped p_world;
    try {
      // Use a short timeout to avoid blocking forever
      auto tf = tf_buffer_.lookupTransform(
        world_frame_, p_cam.header.frame_id, p_cam.header.stamp,
        rclcpp::Duration::from_seconds(0.2)
      );
      tf2::doTransform(p_cam, p_world, tf);
    } catch (const std::exception &e) {
      // TF not ready for this timestamp; skip
      return;
    }

    // Publish detected point
    detected_pub_->publish(p_world);

    // Publish marker (a little sphere)
    visualization_msgs::msg::Marker mk;
    mk.header = p_world.header;
    mk.ns = "lantern";
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.position.x = p_world.point.x;
    mk.pose.position.y = p_world.point.y;
    mk.pose.position.z = p_world.point.z;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.3;
    mk.scale.y = 0.3;
    mk.scale.z = 0.3;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.6;
    mk.color.b = 0.1;
    mk.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker_pub_->publish(mk);
  }

  static double getDepthMedian(const cv::Mat &depth, const std::string &encoding,
                               int u, int v, int radius)
  {
    std::vector<double> vals;
    vals.reserve((2*radius+1)*(2*radius+1));

    const int u0 = std::max(0, u - radius);
    const int v0 = std::max(0, v - radius);
    const int u1 = std::min(depth.cols - 1, u + radius);
    const int v1 = std::min(depth.rows - 1, v + radius);

    if (encoding == "16UC1") {
      for (int yy = v0; yy <= v1; yy++) {
        for (int xx = u0; xx <= u1; xx++) {
          uint16_t d = depth.at<uint16_t>(yy, xx);
          if (d == 0) continue;
          vals.push_back((double)d / 1000.0); // mm -> m
        }
      }
    } else if (encoding == "32FC1") {
      for (int yy = v0; yy <= v1; yy++) {
        for (int xx = u0; xx <= u1; xx++) {
          float d = depth.at<float>(yy, xx);
          if (!std::isfinite(d) || d <= 0.0f) continue;
          vals.push_back((double)d); // already meters
        }
      }
    } else {
      // Unknown depth encoding
      return -1.0;
    }

    if (vals.empty()) return -1.0;
    std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
    return vals[vals.size()/2];
  }

private:
  // Params
  std::string semantic_topic_;
  std::string depth_topic_;
  std::string cam_info_topic_;
  std::string world_frame_;

  int target_b_, target_g_, target_r_;
  int tol_b_, tol_g_, tol_r_;
  int min_area_;
  int depth_window_;
  bool publish_debug_mask_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Pub/Sub
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sem_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

  // Cache
  std::mutex mutex_;
  std::optional<sensor_msgs::msg::CameraInfo> cam_info_;
  std::string cam_info_frame_;

  sensor_msgs::msg::Image::SharedPtr last_depth_msg_;
  std::optional<cv::Mat> last_depth_cv_;
  std::string last_depth_encoding_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
