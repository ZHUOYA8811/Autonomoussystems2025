#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/mat.hpp>

#include <mutex>
#include <vector>
#include <memory>
#include <string>

namespace lantern_detector_pkg {

class LanternDetectorNode : public rclcpp::Node {
public:
  explicit LanternDetectorNode(const rclcpp::NodeOptions& options);

private:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

  struct TrackedLantern {
    geometry_msgs::msg::Point position;
    int count{0};
  };

  void synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                             const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

  void update_lanterns(const geometry_msgs::msg::Point& new_position);
  void publish_lanterns();

  double get_depth_m(const cv::Mat& depth, const std::string& encoding, int u, int v) const;

private:
  // Params
  std::string depth_topic_;
  std::string semantic_topic_;
  std::string depth_info_topic_;
  std::string world_frame_;

  std::string output_topic_;
  std::string count_topic_;
  std::string marker_topic_;

  double min_depth_m_{0.1};
  double max_depth_m_{50.0};
  double distance_threshold_m_{2.0};
  int min_sightings_{30};
  int pixel_stride_{2};
  int sync_queue_size_{30};
  double tf_timeout_s_{0.2};

  double marker_scale_m_{0.5};
  std::vector<double> marker_color_{1.0, 1.0, 0.0, 1.0}; // RGBA

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lantern_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lantern_counts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // CameraInfo
  sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info_;
  std::mutex depth_info_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_info_;

  // Sync subs
  rmw_qos_profile_t sensor_qos_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_semantic_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_filter_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Tracking
  std::vector<TrackedLantern> detected_lanterns_;
};

}  // namespace lantern_detector_pkg