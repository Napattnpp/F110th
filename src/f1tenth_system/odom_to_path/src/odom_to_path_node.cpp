#include <deque>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class OdomToPathNode final : public rclcpp::Node
{
public:
  explicit OdomToPathNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("odom_to_path_node", options)
  {
    odom_topic_      = declare_parameter<std::string>("odom_topic",      "/odom");
    path_topic_      = declare_parameter<std::string>("path_topic",      "/path");
    max_poses_       = declare_parameter<int>("max_poses",               5000);
    min_step_dist_   = declare_parameter<double>("min_step_dist",        0.05);
    publish_every_n_ = declare_parameter<int>("publish_every_n",         5);
    use_odom_stamp_  = declare_parameter<bool>("use_odom_stamp",         true);

    // Clamp invalid parameter values
    if (max_poses_ < 1) {
      RCLCPP_WARN(get_logger(), "max_poses < 1 is invalid, forcing to 1.");
      max_poses_ = 1;
    }
    if (publish_every_n_ < 1) {
      RCLCPP_WARN(get_logger(), "publish_every_n < 1 is invalid, forcing to 1.");
      publish_every_n_ = 1;
    }
    if (min_step_dist_ < 0.0) {
      RCLCPP_WARN(get_logger(), "min_step_dist < 0.0 is invalid, forcing to 0.0.");
      min_step_dist_ = 0.0;
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));

    // Use a deeper queue so bursts from vesc_driver (50 Hz) are not dropped
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(50),
      std::bind(&OdomToPathNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_topic      : %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "path_topic      : %s", path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "publish_every_n : %d", publish_every_n_);
    RCLCPP_INFO(get_logger(), "min_step_dist   : %.3f m", min_step_dist_);
    RCLCPP_INFO(get_logger(), "max_poses       : %d", max_poses_);
    RCLCPP_INFO(get_logger(), "use_odom_stamp  : %s", use_odom_stamp_ ? "true" : "false");
  }

private:
  // ---------------------------------------------------------------------------
  // odomCallback: called every time a new Odometry message arrives
  // ---------------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Set the path frame_id once from the incoming odom frame (usually "odom")
    if (path_msg_.header.frame_id.empty()) {
      path_msg_.header.frame_id =
        msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    // Distance-based downsampling: skip if robot hasn't moved enough
    if (has_last_) {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      if ((dx * dx + dy * dy) < (min_step_dist_ * min_step_dist_)) {
        publishMaybe(msg);
        return;
      }
    }

    // Build a stamped pose and append it to the ring-buffer
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path_msg_.header.frame_id;

    //#> Napattnpp: If-else notation
    if (use_odom_stamp_) { ps.header.stamp = msg->header.stamp; }
    else { ps.header.stamp = now(); }

    ps.pose = msg->pose.pose;

    poses_.push_back(ps);

    // Enforce maximum path length (bounded memory)
    while (static_cast<int>(poses_.size()) > max_poses_) {
      poses_.pop_front();
    }

    last_x_   = x;
    last_y_   = y;
    has_last_ = true;

    publishMaybe(msg);
  }

  // ---------------------------------------------------------------------------
  // publishMaybe: publish path once every publish_every_n_ callbacks
  // ---------------------------------------------------------------------------
  void publishMaybe(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    msg_count_++;
    if ((msg_count_ % static_cast<uint64_t>(publish_every_n_)) != 0) {
      return;
    }

    //#> Napattnpp: If-else notation
    if (use_odom_stamp_) { path_msg_.header.stamp = msg->header.stamp; }
    else { path_msg_.header.stamp = now(); }

    path_msg_.poses.assign(poses_.begin(), poses_.end());
    path_pub_->publish(path_msg_);
  }

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  std::string odom_topic_;
  std::string path_topic_;
  int         max_poses_{5000};
  double      min_step_dist_{0.05};
  int         publish_every_n_{5};
  bool        use_odom_stamp_{true};

  // ---------------------------------------------------------------------------
  // ROS I/O
  // ---------------------------------------------------------------------------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr        path_pub_;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  nav_msgs::msg::Path                              path_msg_;
  std::deque<geometry_msgs::msg::PoseStamped>      poses_;
  bool     has_last_{false};
  double   last_x_{0.0};
  double   last_y_{0.0};
  uint64_t msg_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToPathNode>());
  rclcpp::shutdown();
  return 0;
}
