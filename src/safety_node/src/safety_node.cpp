#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

class SafetyNode : public rclcpp::Node {
public:
    SafetyNode() : Node("safety_node") {
        // Parameters
        this->declare_parameter("ttc_threshold_static", 0.55); // Seconds for static obstacles
        this->declare_parameter("ttc_threshold_dynamic", 0.30); // Seconds for dynamic obstacles
        this->declare_parameter("opponent_size_buffer", 0.35); // Padding around opponent size

        ttc_threshold_static_ = this->get_parameter("ttc_threshold_static").as_double();
        ttc_threshold_dynamic_ = this->get_parameter("ttc_threshold_dynamic").as_double();
        opponent_size_buffer_ = this->get_parameter("opponent_size_buffer").as_double();

        // Setup TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriptions
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SafetyNode::scan_callback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 10, std::bind(&SafetyNode::odom_callback, this, std::placeholders::_1));

        obstacles_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
            "/perception/obstacles", 10, std::bind(&SafetyNode::obstacles_callback, this, std::placeholders::_1));

        waypoints_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", 10, std::bind(&SafetyNode::waypoints_callback, this, std::placeholders::_1));

        aeb_enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/aeb_enabled", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                aeb_enabled_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Automatic AEB Watchdog %s via controller button [0]", aeb_enabled_ ? "ENABLED" : "DISABLED");
            });

        // Publishers
        brake_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/brake", 10);
        pause_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pause_navigation", 10);
        aeb_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/aeb_active", 10);
        aeb_status_pub_ = this->create_publisher<std_msgs::msg::String>("/aeb_status", 10);

        RCLCPP_INFO(this->get_logger(), "Dynamic Safety Node Initialized.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ego_speed_ = msg->twist.twist.linear.x;
    }

    void waypoints_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        global_waypoints_ = msg->wpnts;
    }

    void obstacles_callback(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_obstacles_ = msg->obstacles;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!aeb_enabled_) {
            release_brake();
            return;
        }

        double current_speed = ego_speed_;
        auto obstacles = latest_obstacles_;
        auto waypoints = global_waypoints_;

        if (waypoints.empty()) {
            // If waypoints are not loaded yet, default to static AEB model
            run_static_aeb(msg, current_speed);
            return;
        }

        // Struct to hold tracked opponent info in vehicle frame
        struct DynamicOpponent {
            double x; // Position in laser/vehicle frame
            double y;
            double vx; // Velocity in laser/vehicle frame
            double vy;
            double radius;
            double angle_min;
            double angle_max;
        };

        std::vector<DynamicOpponent> dynamic_opponents;

        // Transform obstacles from map frame to scan frame
        std::string target_frame = msg->header.frame_id;
        std::string source_frame = "map";

        geometry_msgs::msg::TransformStamped transform_stamped;
        bool tf_ok = false;
        try {
            transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            tf_ok = true;
        } catch (tf2::TransformException &ex) {
            // Fallback to static if TF lookup fails temporarily
            run_static_aeb(msg, current_speed);
            return;
        }

        for (const auto &obs : obstacles) {
            if (obs.is_static) continue; // Skip static obstacles (dealt with via static scan rate)

            // Find nearest waypoint to get heading angle (psi)
            double track_length = waypoints.back().s_m;
            double psi = 0.0;
            double min_dist = std::numeric_limits<double>::max();
            for (const auto &wp : waypoints) {
                double dist = std::abs(wp.s_m - obs.s_center);
                if (dist > track_length / 2.0) {
                    dist = track_length - dist;
                }
                if (dist < min_dist) {
                    min_dist = dist;
                    psi = wp.psi_rad;
                }
            }

            // Opponent absolute velocity vector in map Cartesian coordinates
            double v_opp_map_x = obs.vs * std::cos(psi) - obs.vd * std::sin(psi);
            double v_opp_map_y = obs.vs * std::sin(psi) + obs.vd * std::cos(psi);

            // Transform center position from map to vehicle frame
            geometry_msgs::msg::PointStamped point_in_map;
            point_in_map.header.frame_id = source_frame;
            point_in_map.point.x = obs.s_center; // Wait, s_center is Frenet coordinate. 
            // Oh, we should use the Cartesian map coordinate! Let's lookup waypoint at s_center to get X and Y.
            double obs_map_x = 0.0;
            double obs_map_y = 0.0;
            double min_s_dist = std::numeric_limits<double>::max();
            for (const auto &wp : waypoints) {
                double dist = std::abs(wp.s_m - obs.s_center);
                if (dist > track_length / 2.0) {
                    dist = track_length - dist;
                }
                if (dist < min_s_dist) {
                    min_s_dist = dist;
                    obs_map_x = wp.x_m - obs.d_center * std::sin(wp.psi_rad);
                    obs_map_y = wp.y_m + obs.d_center * std::cos(wp.psi_rad);
                }
            }

            point_in_map.point.x = obs_map_x;
            point_in_map.point.y = obs_map_y;
            point_in_map.point.z = 0.0;

            geometry_msgs::msg::PointStamped point_in_car;
            tf2::doTransform(point_in_map, point_in_car, transform_stamped);

            // Transform velocity vector to vehicle frame (rotation only)
            // We construct a quaternion from transform_stamped orientation
            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w
            );
            tf2::Matrix3x3 rot(q);
            tf2::Vector3 vel_map(v_opp_map_x, v_opp_map_y, 0.0);
            tf2::Vector3 vel_car = rot * vel_map;

            double radius = (obs.size / 2.0) + opponent_size_buffer_;
            double dist_to_opp = std::hypot(point_in_car.point.x, point_in_car.point.y);
            if (dist_to_opp < radius) continue; // Inside radius, skip or handle

            double center_angle = std::atan2(point_in_car.point.y, point_in_car.point.x);
            double half_spread = std::asin(radius / dist_to_opp);

            DynamicOpponent opp;
            opp.x = point_in_car.point.x;
            opp.y = point_in_car.point.y;
            opp.vx = vel_car.x();
            opp.vy = vel_car.y();
            opp.radius = radius;
            opp.angle_min = center_angle - half_spread;
            opp.angle_max = center_angle + half_spread;

            dynamic_opponents.push_back(opp);
        }

        // Evaluate LaserScan beams
        bool collision_detected = false;
        double min_ttc = std::numeric_limits<double>::max();

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            if (std::isnan(r) || r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;

            // Check if this beam falls inside any opponent's angular sector
            bool is_hitting_opponent = false;
            DynamicOpponent hit_opp;
            for (const auto &opp : dynamic_opponents) {
                if (angle >= opp.angle_min && angle <= opp.angle_max) {
                    is_hitting_opponent = true;
                    hit_opp = opp;
                    break;
                }
            }

            double r_dot = 0.0;
            double ttc_threshold = ttc_threshold_static_;

            if (is_hitting_opponent) {
                // Opponent is moving, evaluate relative velocity projection
                double v_rel_x = current_speed - hit_opp.vx;
                double v_rel_y = -hit_opp.vy;
                r_dot = v_rel_x * std::cos(angle) + v_rel_y * std::sin(angle);
                ttc_threshold = ttc_threshold_dynamic_;
            } else {
                // Wall is static, standard range rate
                r_dot = current_speed * std::cos(angle);
            }

            if (r_dot > 0.0) {
                double ttc = r / r_dot;
                if (ttc < ttc_threshold) {
                    collision_detected = true;
                    min_ttc = std::min(min_ttc, ttc);
                }
            }
        }

        if (collision_detected) {
            trigger_brake();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "DYNAMIC AEB TRIGGERED! Min TTC: %f seconds", min_ttc);
        } else {
            release_brake();
        }
    }

    void run_static_aeb(const sensor_msgs::msg::LaserScan::SharedPtr msg, double current_speed) {
        bool collision_detected = false;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            if (std::isnan(r) || r < msg->range_min || r > msg->range_max) continue;

            double angle = msg->angle_min + i * msg->angle_increment;
            double r_dot = current_speed * std::cos(angle);
            if (r_dot > 0.0) {
                double ttc = r / r_dot;
                if (ttc < ttc_threshold_static_) {
                    collision_detected = true;
                    break;
                }
            }
        }

        if (collision_detected) {
            trigger_brake();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "STATIC AEB TRIGGERED!");
        } else {
            release_brake();
        }
    }

    void trigger_brake() {
        // Publish AckermannDriveStamped speed = 0.0 to /brake
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "ego_racecar/laser";
        drive_msg.drive.speed = 0.0;
        drive_msg.drive.steering_angle = 0.0;
        brake_pub_->publish(drive_msg);

        // Publish True to /pause_navigation lock
        std_msgs::msg::Bool pause_msg;
        pause_msg.data = true;
        pause_pub_->publish(pause_msg);

        // Publish AEB active and status
        std_msgs::msg::Bool active_msg;
        active_msg.data = true;
        aeb_active_pub_->publish(active_msg);

        std_msgs::msg::String status_msg;
        status_msg.data = "TRIGGERED";
        aeb_status_pub_->publish(status_msg);
    }

    void release_brake() {
        // Publish False to /pause_navigation lock to keep navigation free
        std_msgs::msg::Bool pause_msg;
        pause_msg.data = false;
        pause_pub_->publish(pause_msg);

        // Publish AEB inactive and status
        std_msgs::msg::Bool active_msg;
        active_msg.data = false;
        aeb_active_pub_->publish(active_msg);

        std_msgs::msg::String status_msg;
        status_msg.data = "SAFE";
        aeb_status_pub_->publish(status_msg);
    }

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aeb_enabled_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aeb_active_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr aeb_status_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Variables
    double ego_speed_ = 0.0;
    bool aeb_enabled_ = true;
    std::vector<f110_msgs::msg::Wpnt> global_waypoints_;
    std::vector<f110_msgs::msg::Obstacle> latest_obstacles_;
    std::mutex mutex_;

    // Thresholds
    double ttc_threshold_static_;
    double ttc_threshold_dynamic_;
    double opponent_size_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
