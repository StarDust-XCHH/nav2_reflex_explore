#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <unordered_set>
#include <chrono>

static inline double sqr(double x) { return x * x; }

class CoverageWandererNode : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  CoverageWandererNode()
  : Node("coverage_wanderer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {

    // Parameters
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("robot_base_frame", "base_link");
    declare_parameter<double>("goal_radius_m", 0.5);
    declare_parameter<double>("visited_radius_m", 0.6);
    declare_parameter<int>("max_goal_history", 5);               // 新增参数
    declare_parameter<double>("goal_exclusion_radius_m", 1.0);   // 新增参数

    get_parameter("global_frame", global_frame_);
    get_parameter("robot_base_frame", robot_base_frame_);
    goal_radius_m_ = get_parameter("goal_radius_m").as_double();
    visited_radius_m_ = get_parameter("visited_radius_m").as_double();
    max_goal_history_ = get_parameter("max_goal_history").as_int();
    goal_exclusion_radius_m_ = get_parameter("goal_exclusion_radius_m").as_double();

    // Subscribers & Clients
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local(),
      std::bind(&CoverageWandererNode::mapCallback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
    nav_client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "Nav2 ready.");

    // Services
    pause_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/pause_wandering",
      std::bind(&CoverageWandererNode::handlePause, this, std::placeholders::_1, std::placeholders::_2));
    resume_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/resume_wandering",
      std::bind(&CoverageWandererNode::handleResume, this, std::placeholders::_1, std::placeholders::_2));

    // Timer: mark current pose as visited periodically
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CoverageWandererNode::timerCallback, this));
  }

private:
  // ===== ROS Interfaces =====
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  // ===== State =====
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  bool paused_{false};
  bool navigating_{false};

  // Visited set: store (x,y) in map indices
  struct PoseHash {
    size_t operator()(const std::pair<int, int>& p) const {
      return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
  };
  std::unordered_set<std::pair<int, int>, PoseHash> visited_cells_;

  // Recent goal history (world coordinates)
  std::deque<std::pair<double, double>> recent_goals_;


  // ===== Parameters =====
  std::string global_frame_, robot_base_frame_;
  double goal_radius_m_;
  double visited_radius_m_;
  int max_goal_history_;
  double goal_exclusion_radius_m_;

  // ===== Helpers =====
  std::optional<std::pair<double, double>> getRobotPose() {
    try {
      auto tf = tf_buffer_.lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
      return std::make_pair(tf.transform.translation.x, tf.transform.translation.y);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  std::pair<int, int> worldToMap(double x, double y) const {
    const auto& info = map_->info;
    int mx = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
    int my = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
    return {mx, my};
  }

  std::pair<double, double> mapToWorld(int mx, int my) const {
    const auto& info = map_->info;
    double x = info.origin.position.x + (mx + 0.5) * info.resolution;
    double y = info.origin.position.y + (my + 0.5) * info.resolution;
    return {x, y};
  }

  bool isFree(int mx, int my) const {
    if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
        my < 0 || my >= static_cast<int>(map_->info.height)) return false;
    return map_->data[my * map_->info.width + mx] == 0;
  }

  void markVisited(double x, double y) {
    auto [cx, cy] = worldToMap(x, y);
    int radius = static_cast<int>(std::ceil(visited_radius_m_ / map_->info.resolution));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx*dx + dy*dy > radius*radius) continue;
        visited_cells_.insert({cx + dx, cy + dy});
      }
    }
  }

  bool isVisited(int mx, int my) const {
    return visited_cells_.count({mx, my}) > 0;
  }

  std::optional<std::pair<int, int>> findBestUnvisitedCell() {
    if (!map_) return std::nullopt;

    auto robot = getRobotPose();
    if (!robot) return std::nullopt;

    auto [rx_map, ry_map] = worldToMap(robot->first, robot->second);
    double search_radius_m = 8.0;
    int search_radius_cells = static_cast<int>(std::ceil(search_radius_m / map_->info.resolution));

    std::vector<std::pair<int, int>> candidate_cells;

    for (int dy = -search_radius_cells; dy <= search_radius_cells; ++dy) {
      for (int dx = -search_radius_cells; dx <= search_radius_cells; ++dx) {
        int mx = rx_map + dx;
        int my = ry_map + dy;

        if (mx < 0 || mx >= static_cast<int>(map_->info.width) ||
            my < 0 || my >= static_cast<int>(map_->info.height)) continue;

        if (!isFree(mx, my) || isVisited(mx, my)) continue;

        // === 排除靠近 recent_goals_ 的点 ===
        auto [wx, wy] = mapToWorld(mx, my);
        bool too_close = false;
        for (const auto& [gx, gy] : recent_goals_) {
          if (sqr(wx - gx) + sqr(wy - gy) < sqr(goal_exclusion_radius_m_)) {
            too_close = true;
            break;
          }
        }
        if (too_close) continue;

        candidate_cells.emplace_back(mx, my);
      }
    }

    if (candidate_cells.empty()) {
      return std::nullopt;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, candidate_cells.size() - 1);
    return candidate_cells[dis(gen)];
  }

  void sendNavigationGoal(double x, double y) {
    // 记录目标到历史（用于后续排斥）
    recent_goals_.push_back({x, y});  // deque 的 push_back == queue 的 push
    while (static_cast<int>(recent_goals_.size()) > max_goal_history_) {
        recent_goals_.pop_front();      // deque 支持 pop_front
    }

    auto robot = getRobotPose();
    if (!robot) return;

    double yaw = std::atan2(y - robot->second, x - robot->first);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf2::Quaternion q; q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    NavigateToPose::Goal goal;
    goal.pose = pose;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback = [this](const GoalHandle::WrappedResult& result) {
      navigating_ = false;
      RCLCPP_INFO(this->get_logger(), "Wander goal completed with code %d", static_cast<int>(result.code));

      if (!paused_) {
        this->trySendNextGoal();
      }
    };

    RCLCPP_INFO(this->get_logger(), "Sending wander goal: (%.2f, %.2f)", x, y);
    navigating_ = true;
    nav_client_->async_send_goal(goal, opts);
  }

  void trySendNextGoal() {
    if (paused_ || !map_ || navigating_) return;

    auto robot = getRobotPose();
    if (!robot) return;

    markVisited(robot->first, robot->second);

    auto best_cell = findBestUnvisitedCell();
    if (!best_cell) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No valid wander target found.");
      return;
    }

    auto [wx, wy] = mapToWorld(best_cell->first, best_cell->second);
    sendNavigationGoal(wx, wy);
  }

  // ===== Callbacks =====
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Map received. Starting coverage wandering.");

    if (!paused_ && !navigating_) {
      trySendNextGoal();
    }
  }

  void handlePause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    paused_ = true;
    if (navigating_) {
      nav_client_->async_cancel_all_goals();
      navigating_ = false;
    }
    res->success = true;
    res->message = "Wandering paused.";
    RCLCPP_INFO(this->get_logger(), "Wandering PAUSED.");
  }

  void handleResume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    paused_ = false;
    res->success = true;
    res->message = "Wandering resumed.";
    RCLCPP_INFO(this->get_logger(), "Wandering RESUMED.");

    if (!navigating_) {
      trySendNextGoal();
    }
  }

  void timerCallback() {
    if (paused_ || !map_) return;
    auto robot = getRobotPose();
    if (robot) {
      markVisited(robot->first, robot->second);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoverageWandererNode>());
  rclcpp::shutdown();
  return 0;
}