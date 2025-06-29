#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <angles/angles.h>
#include <vector>
#include <tuple>
#include <limits>
#include <random>

using std::placeholders::_1;

// Goal struct holds the navigation goal state
struct Goal {
  double x = 0.0, y = 0.0;
  bool available = false;
  bool reached = false;
};

// Global variables for goal, odometry, and laser scan data
Goal goal;
nav_msgs::msg::Odometry::SharedPtr odom_data;
sensor_msgs::msg::LaserScan::SharedPtr scan_data;

// Main class implementing the DWA Planner
class DWAPlanner : public rclcpp::Node {
public:
  DWAPlanner() : Node("dwa_planner") {
    // Subscriptions for goal pose, odometry, and laser scan
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&DWAPlanner::goal_callback, this, _1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DWAPlanner::odom_callback, this, _1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DWAPlanner::scan_callback, this, _1));

    // Publishers for velocity command and path visualization
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visual_paths", 10);
    best_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/best_path", 10);

    // Parameters for motion
    max_speed_ = 0.3;
    max_turn_ = 2.5;
    step_time_ = 0.1;

    // Timer to periodically run the DWA algorithm
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(step_time_),
      std::bind(&DWAPlanner::movement_loop, this));
  }

private:
  // ROS2 interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_speed_, max_turn_, step_time_;

  // Callback for goal pose
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.reached = false;
    goal.available = true;
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", goal.x, goal.y);
  }

  // Callback for odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_data = msg;
  }

  // Callback for laser scan data
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_data = msg;
  }

  // Predict future trajectory from current position with given speed and turn rate
  std::vector<std::pair<double, double>> predict_motion(double speed, double turn_rate) {
    std::vector<std::pair<double, double>> path;
    if (!odom_data) return path;

    double x = odom_data->pose.pose.position.x;
    double y = odom_data->pose.pose.position.y;

    // Extract yaw from quaternion
    tf2::Quaternion q(
      odom_data->pose.pose.orientation.x,
      odom_data->pose.pose.orientation.y,
      odom_data->pose.pose.orientation.z,
      odom_data->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Simulate forward motion using unicycle model
    for (int i = 0; i < 30; ++i) {
      yaw += turn_rate * step_time_;
      x += speed * cos(yaw) * step_time_;
      y += speed * sin(yaw) * step_time_;
      path.emplace_back(x, y);
    }
    return path;
  }

  // Check if any point in path collides with an obstacle
  int check_for_collisions(const std::vector<std::pair<double, double>>& path) {
    if (!scan_data || !odom_data) return -100000;

    double safety_radius = 0.25;

    // Get robot's position and orientation
    double rx = odom_data->pose.pose.position.x;
    double ry = odom_data->pose.pose.position.y;

    tf2::Quaternion q(
      odom_data->pose.pose.orientation.x,
      odom_data->pose.pose.orientation.y,
      odom_data->pose.pose.orientation.z,
      odom_data->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Check every point in path for proximity to obstacles
    for (const auto& [gx, gy] : path) {
      double dx = gx - rx;
      double dy = gy - ry;

      // Convert to laser frame
      double x = std::cos(-yaw) * dx - std::sin(-yaw) * dy;
      double y = std::sin(-yaw) * dx + std::cos(-yaw) * dy;

      double dist = std::hypot(x, y);
      double angle = std::atan2(y, x);

      int index = static_cast<int>((angle - scan_data->angle_min) / scan_data->angle_increment);
      if (index < 0 || index >= static_cast<int>(scan_data->ranges.size())) continue;

      double scan_range = scan_data->ranges[index];
      if (!std::isfinite(scan_range)) continue;

      if (scan_range < dist + safety_radius) return -100000; // Hard collision
      if (scan_range - dist < 0.1) return -500;              // Near-collision
    }
    return 0; // Safe path
  }

  // Evaluate a path using multiple heuristics
  double evaluate_path(const std::vector<std::pair<double, double>>& path, double speed, double turn) {
    if (!odom_data) return -1e6;

    double cx = odom_data->pose.pose.position.x;
    double cy = odom_data->pose.pose.position.y;

    tf2::Quaternion q(
      odom_data->pose.pose.orientation.x,
      odom_data->pose.pose.orientation.y,
      odom_data->pose.pose.orientation.z,
      odom_data->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Heuristic components
    double goal_dist = -std::hypot(path.back().first - goal.x, path.back().second - goal.y);  // prefer shorter dist
    double angle_to_goal = std::atan2(goal.y - cy, goal.x - cx);
    double heading_score = -std::abs(angles::normalize_angle(angle_to_goal - yaw));           // prefer aligned heading
    double collision_score = check_for_collisions(path);                                       // penalize collision
    double smoothness_score = -0.05 * std::abs(turn);                                          // penalize sharp turns

    double total = 10 * goal_dist + 5 * heading_score + collision_score + smoothness_score;
    return total;
  }

  // Main control loop to generate, evaluate, and select best path
  void movement_loop() {
    if (!odom_data || !scan_data || !goal.available || goal.reached) return;

    // Check if goal is reached
    double dx = goal.x - odom_data->pose.pose.position.x;
    double dy = goal.y - odom_data->pose.pose.position.y;
    double distance_to_goal = std::hypot(dx, dy);

    if (distance_to_goal < 0.1) {
      goal.reached = true;
      cmd_pub_->publish(geometry_msgs::msg::Twist()); // stop
      RCLCPP_INFO(this->get_logger(), "Goal reached.");
      return;
    }

    // Sample random velocity commands
    std::vector<std::tuple<double, double, std::vector<std::pair<double, double>>>> paths;
    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<double> speed_dist(0.0, max_speed_);
    std::uniform_real_distribution<double> turn_dist(-max_turn_, max_turn_);

    for (int i = 0; i < 300; ++i) {
      double speed = speed_dist(rng);
      double turn = turn_dist(rng);
      if (speed < 0.05 && std::abs(turn) < 0.2) continue; // skip near-zero motion
      auto path = predict_motion(speed, turn);
      paths.emplace_back(speed, turn, path);
    }

    // Select the best-scored path
    std::pair<double, double> best_cmd;
    std::vector<std::pair<double, double>> best_path;
    double best_score = -std::numeric_limits<double>::infinity();

    for (const auto& [speed, turn, path] : paths) {
      double score = evaluate_path(path, speed, turn);
      if (score > best_score) {
        best_score = score;
        best_cmd = {speed, turn};
        best_path = path;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Best command: speed=%.2f, turn=%.2f, score=%.2f", best_cmd.first, best_cmd.second, best_score);
    // Publish selected command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = best_cmd.first;
    cmd.angular.z = best_cmd.second;
    cmd_pub_->publish(cmd);

    // Publish all path visualizations (faint green lines)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "dwa_paths";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.g = 0.8f;
    marker.color.a = 0.3f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto& [_, __, path] : paths) {
      for (size_t i = 1; i < path.size(); ++i) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = path[i - 1].first;
        p1.y = path[i - 1].second;
        p2.x = path[i].first;
        p2.y = path[i].second;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }
    path_pub_->publish(marker);

    // Publish best path (blue line)
    visualization_msgs::msg::Marker best_marker;
    best_marker.header.frame_id = "odom";
    best_marker.header.stamp = this->now();
    best_marker.ns = "dwa_best_path";
    best_marker.id = 1;
    best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    best_marker.action = visualization_msgs::msg::Marker::ADD;
    best_marker.scale.x = 0.03;
    best_marker.color.b = 1.0f;
    best_marker.color.a = 1.0f;
    best_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto& [x, y] : best_path) {
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      best_marker.points.push_back(pt);
    }
    best_path_pub_->publish(best_marker);
  }
};

// Main entry point
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DWAPlanner>());
  rclcpp::shutdown();
  return 0;
}
