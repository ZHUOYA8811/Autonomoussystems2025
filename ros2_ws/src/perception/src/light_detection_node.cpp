#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int32.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <optional>
#include <limits>
#include <cstdint>
#include <fstream>
#include <vector>
#include <array>
#include <ctime>
#include <iomanip>
#include <sstream>

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

    // Detection count monitor
    count_topic_ = this->declare_parameter<std::string>("count_topic", "/lantern_detection_count");
    count_log_interval_sec_ = this->declare_parameter<double>("count_log_interval_sec", 2.0);

    // ---------- Lantern position logging ----------
    log_file_path_ = this->declare_parameter<std::string>(
      "log_file_path", "/tmp/lantern_detections.txt");
    // Minimum distance (m) from all confirmed positions to count as a *new* lantern
    // 7m: 在防重复和不合并不同灯笼之间取平衡
    // （之前10m太大，会把8m内的两个不同灯笼合并成一个）
    dedup_radius_ = this->declare_parameter<double>("dedup_radius", 7.0);
    // Maximum number of unique lanterns to record (0 = unlimited)
    max_unique_lanterns_ = this->declare_parameter<int>("max_unique_lanterns", 0);
    // Max depth at which a detection is accepted (farther = too noisy)
    // 12m: 减小最大检测深度以降低X轴噪声
    // （之前18m时远距离检测的X轴误差可达3m+，容易导致同一灯笼被计数两次）
    // 12m内深度噪声<1m，位置精度足够可靠
    max_detect_depth_ = this->declare_parameter<double>("max_detect_depth", 12.0);
    // Number of consecutive frames needed to confirm a new lantern
    // 2帧: 减少确认所需帧数，避免快速飞越时第一个灯笼因在视野内停留不足帧数而漏检
    // dedup_radius仍保持7m，消重逻辑不变，只是降低插入门槛
    min_confirm_count_ = this->declare_parameter<int>("min_confirm_count", 2);
    // Radius within which candidate detections cluster together
    // 3m: 允许深度抖动，但不会把相邻灯笼合并
    // （之前5m太大，两个距离4m的灯笼候选会被错误合并）
    confirm_radius_ = this->declare_parameter<double>("confirm_radius", 3.0);

    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (log_file_.is_open()) {
      // Header
      log_file_ << "# Lantern detections - world frame: " << world_frame_ << "\n";
      log_file_ << "# Columns: index  timestamp_sec  x  y  z\n";
      log_file_.flush();
      RCLCPP_INFO(this->get_logger(), "Logging lantern positions to: %s", log_file_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to open log file: %s", log_file_path_.c_str());
    }

    // ---------- Publishers ----------
    detected_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);
    marker_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("/lantern_marker", 10);
    count_pub_    = this->create_publisher<std_msgs::msg::Int32>(count_topic_, 10);

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
    RCLCPP_INFO(this->get_logger(), "count_topic   : %s", count_topic_.c_str());

    detection_count_ = 0;
    last_count_log_time_ = this->now();
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
    // Reject detections beyond max_detect_depth_ — depth noise grows quadratically
    // with distance, causing the same lantern to appear at wildly different 3D positions
    if (z_m > max_detect_depth_) {
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

    // ---- Candidate / Confirmed lantern pipeline ----
    // Flow: raw detection -> match confirmed? (update avg) -> match candidate? (accumulate)
    //       -> new candidate -> candidate.count >= min_confirm_count_? -> PROMOTE to confirmed
    //
    // This prevents distant noisy first-sightings from immediately registering as a lantern,
    // which was causing the same physical lantern to be counted twice (once at long range
    // with large depth error, once at close range with a different 3D position).

    const double px = p_world.point.x;
    const double py = p_world.point.y;
    const double pz = p_world.point.z;

    // --- Step 1: Does this detection match an already-CONFIRMED lantern? ---
    // 使用X轴加权距离：深度相机的X轴噪声最大，降低其权重以减少重复计数
    int matched_confirmed = -1;
    for (int ci = 0; ci < static_cast<int>(unique_lanterns_.size()); ++ci) {
      double dx = unique_lanterns_[ci][0] - px;
      double dy = unique_lanterns_[ci][1] - py;
      double dz = unique_lanterns_[ci][2] - pz;
      // X轴权重0.5（噪声大时实际偏差可能是计算值的2倍）
      double weighted_dist = std::sqrt(0.5*dx*dx + dy*dy + dz*dz);
      if (weighted_dist < dedup_radius_) {
        matched_confirmed = ci;
        break;
      }
    }

    if (matched_confirmed >= 0) {
      // Update confirmed position with exponential moving average (alpha = 1/(n+1))
      int& n = lantern_obs_count_[matched_confirmed];
      n++;
      double alpha = 1.0 / static_cast<double>(n);
      unique_lanterns_[matched_confirmed][0] += alpha * (px - unique_lanterns_[matched_confirmed][0]);
      unique_lanterns_[matched_confirmed][1] += alpha * (py - unique_lanterns_[matched_confirmed][1]);
      unique_lanterns_[matched_confirmed][2] += alpha * (pz - unique_lanterns_[matched_confirmed][2]);

      // Publish the refined position so downstream (state_machine) gets the best estimate
      geometry_msgs::msg::PointStamped refined;
      refined.header = p_world.header;
      refined.point.x = unique_lanterns_[matched_confirmed][0];
      refined.point.y = unique_lanterns_[matched_confirmed][1];
      refined.point.z = unique_lanterns_[matched_confirmed][2];
      detected_pub_->publish(refined);
    } else {
      // --- Step 2: Does this detection match an existing CANDIDATE? ---
      int matched_cand = -1;
      const auto now_time = this->now();

      // Expire stale candidates (not seen for > 6 seconds)
      // 之前3秒太短：急转弯后飞机可能2-3秒后又看到同一灯笼，但候选已过期
      // 延长到6秒，让转弯后还能继续积累之前的候选
      for (int ci = static_cast<int>(candidates_.size()) - 1; ci >= 0; --ci) {
        if ((now_time - candidates_[ci].last_seen).seconds() > 6.0) {
          candidates_.erase(candidates_.begin() + ci);
        }
      }

      for (int ci = 0; ci < static_cast<int>(candidates_.size()); ++ci) {
        double avg_x = candidates_[ci].sum_x / candidates_[ci].count;
        double avg_y = candidates_[ci].sum_y / candidates_[ci].count;
        double avg_z = candidates_[ci].sum_z / candidates_[ci].count;
        double dx = avg_x - px, dy = avg_y - py, dz = avg_z - pz;
        // X轴加权距离（与Step1一致）
        double weighted_dist = std::sqrt(0.5*dx*dx + dy*dy + dz*dz);
        if (weighted_dist < confirm_radius_) {
          matched_cand = ci;
          break;
        }
      }

      if (matched_cand >= 0) {
        // Accumulate into existing candidate
        auto& cand = candidates_[matched_cand];
        cand.sum_x += px; cand.sum_y += py; cand.sum_z += pz;
        cand.count++;
        cand.last_x = px; cand.last_y = py; cand.last_z = pz;
        cand.last_seen = now_time;

        // Check if candidate is ready to be promoted
        if (cand.count >= min_confirm_count_) {
          // Already at max unique lanterns?
          if (max_unique_lanterns_ > 0 &&
              static_cast<int>(unique_lanterns_.size()) >= max_unique_lanterns_) {
            // Don't promote, just discard
          } else {
            double cx = cand.sum_x / cand.count;
            double cy = cand.sum_y / cand.count;
            double cz = cand.sum_z / cand.count;

            // Final dedup check against all confirmed lanterns (averaged position)
            bool too_close = false;
            for (const auto& lp : unique_lanterns_) {
              double dx2 = lp[0] - cx, dy2 = lp[1] - cy, dz2 = lp[2] - cz;
              // X轴加权距离（与Step1一致）
              double weighted_dist2 = std::sqrt(0.5*dx2*dx2 + dy2*dy2 + dz2*dz2);
              if (weighted_dist2 < dedup_radius_) {
                too_close = true;
                break;
              }
            }

            if (!too_close) {
              unique_lanterns_.push_back({cx, cy, cz});
              lantern_obs_count_.push_back(cand.count);
              const int idx = static_cast<int>(unique_lanterns_.size());

              // Write to file
              if (log_file_.is_open()) {
                log_file_ << std::fixed << std::setprecision(4)
                          << idx << "  "
                          << p_world.header.stamp.sec << "." << std::setw(9) << std::setfill('0')
                          << p_world.header.stamp.nanosec << "  "
                          << cx << "  " << cy << "  " << cz
                          << "  (confirmed after " << cand.count << " frames)\n";
                log_file_.flush();
              }

              RCLCPP_INFO(this->get_logger(),
                "[lantern #%d] CONFIRMED at (%.3f, %.3f, %.3f) after %d frames. total=%d",
                idx, cx, cy, cz, cand.count, idx);

              // Publish confirmed detection
              geometry_msgs::msg::PointStamped confirmed_pt;
              confirmed_pt.header = p_world.header;
              confirmed_pt.point.x = cx;
              confirmed_pt.point.y = cy;
              confirmed_pt.point.z = cz;
              detected_pub_->publish(confirmed_pt);
            }
          }
          // Remove promoted/rejected candidate
          candidates_.erase(candidates_.begin() + matched_cand);
        }
      } else {
        // --- Step 3: Start a new candidate ---
        // But only if not already at max and not too close to a confirmed one
        bool near_confirmed = false;
        for (const auto& lp : unique_lanterns_) {
          double dx = lp[0] - px, dy = lp[1] - py, dz = lp[2] - pz;
          // X轴加权距离（与Step1一致）
          double weighted_dist = std::sqrt(0.5*dx*dx + dy*dy + dz*dz);
          if (weighted_dist < dedup_radius_) {
            near_confirmed = true;
            break;
          }
        }
        if (!near_confirmed) {
          Candidate c;
          c.sum_x = px; c.sum_y = py; c.sum_z = pz;
          c.count = 1;
          c.last_x = px; c.last_y = py; c.last_z = pz;
          c.last_seen = now_time;
          candidates_.push_back(c);
          RCLCPP_DEBUG(this->get_logger(),
            "New lantern candidate at (%.2f, %.2f, %.2f)", px, py, pz);
        }
      }
    }

    // Count monitor: publish current total detections
    detection_count_++;
    std_msgs::msg::Int32 count_msg;
    count_msg.data = (detection_count_ > static_cast<std::uint64_t>(std::numeric_limits<int32_t>::max()))
      ? std::numeric_limits<int32_t>::max()
      : static_cast<int32_t>(detection_count_);
    count_pub_->publish(count_msg);

    const auto now = this->now();
    if (count_log_interval_sec_ > 0.0 &&
        (now - last_count_log_time_).seconds() >= count_log_interval_sec_) {
      RCLCPP_INFO(this->get_logger(), "Lantern detection count: %lld",
                  static_cast<long long>(detection_count_));
      last_count_log_time_ = now;
    }

    // Publish marker (a little sphere) — one persistent marker per unique lantern
    // Re-publish all known unique lanterns every frame so they stay visible in RViz.
    for (int li = 0; li < static_cast<int>(unique_lanterns_.size()); ++li) {
      visualization_msgs::msg::Marker mk;
      mk.header = p_world.header;
      mk.ns = "lantern";
      mk.id = li;   // unique id per lantern — no longer overwriting same marker
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = unique_lanterns_[li][0];
      mk.pose.position.y = unique_lanterns_[li][1];
      mk.pose.position.z = unique_lanterns_[li][2];
      mk.pose.orientation.w = 1.0;
      mk.scale.x = 0.4;
      mk.scale.y = 0.4;
      mk.scale.z = 0.4;
      mk.color.a = 1.0;
      mk.color.r = 1.0;
      mk.color.g = 0.6;
      mk.color.b = 0.1;
      mk.lifetime = rclcpp::Duration::from_seconds(1.0);
      marker_pub_->publish(mk);

      // Text label above the sphere
      visualization_msgs::msg::Marker txt;
      txt.header = mk.header;
      txt.ns = "lantern_label";
      txt.id = li;
      txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = unique_lanterns_[li][0];
      txt.pose.position.y = unique_lanterns_[li][1];
      txt.pose.position.z = unique_lanterns_[li][2] + 0.5;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.35;
      txt.color.a = 1.0; txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0;
      txt.lifetime = rclcpp::Duration::from_seconds(1.0);
      std::ostringstream oss;
      oss << "#" << (li+1) << " (" << std::fixed << std::setprecision(1)
          << unique_lanterns_[li][0] << ","
          << unique_lanterns_[li][1] << ","
          << unique_lanterns_[li][2] << ")";
      txt.text = oss.str();
      marker_pub_->publish(txt);
    }
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
  std::string count_topic_;

  int target_b_, target_g_, target_r_;
  int tol_b_, tol_g_, tol_r_;
  int min_area_;
  int depth_window_;
  bool publish_debug_mask_;
  double count_log_interval_sec_;

  // Monitor state
  std::uint64_t detection_count_{0};
  rclcpp::Time last_count_log_time_;

  // Lantern position logging
  std::string log_file_path_;
  double dedup_radius_;
  int max_unique_lanterns_;
  double max_detect_depth_;       // reject detections farther than this (m)
  int min_confirm_count_;         // frames needed to promote candidate -> confirmed
  double confirm_radius_;         // max jitter (m) for a candidate cluster
  std::ofstream log_file_;
  std::vector<std::array<double, 3>> unique_lanterns_;  // confirmed lantern positions
  std::vector<int>                   lantern_obs_count_; // observations per confirmed lantern (for avg)

  // Candidate tracking: accumulate detections before confirming as new lantern
  struct Candidate {
    double sum_x{0}, sum_y{0}, sum_z{0};
    int count{0};
    double last_x{0}, last_y{0}, last_z{0};
    rclcpp::Time last_seen;
  };
  std::vector<Candidate> candidates_;

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
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr count_pub_;

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
