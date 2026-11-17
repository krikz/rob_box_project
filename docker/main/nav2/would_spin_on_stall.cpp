// #4475: узел дерева «нужен разворот на месте до свободного курса».
// Угол считает rob_box_teleop.turn_in_place.free_spin (pytest). Здесь только
// чтение пути, скана в base_footprint и нулевого сдвига и вызов той же функции.
//
// SUCCESS — путь не пуст, с начала цели сдвига нет, free_spin вернул угол:
// он записан в {turn_spin_dist}, следом дерево шлёт Spin, затем FollowPath.
// FAILURE — Spin не слать. Дерево ставит 0, Humble Spin на 0 рад сразу успешен.
//
// Сдвиг — не новый порог. Первый тик запоминает позу. Пока база не ушла дальше
// клетки costmap (0.05 м, resolution в nav2_params), прогресс нулевой. Поехал —
// FAILURE, FollowPath не перебивается. halt() сбрасывает эту позу: дерево
// одно на configure, без сброса вторая цель видит сдвиг от первой.
// Точки скана и путь — в base_footprint (лидар не в базе, lslidar_n10 x≈−0.17).
// free_spin поэтому с позой (0,0,0): map-yaw разворачивает стену за спину.

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// Humble: Buffer::transform<PoseStamped> берёт tf2::getTimestamp / getFrameId /
// doTransform. Без этой специализации g++ оставляет их undefined, и bt_navigator
// не грузит .so (Could not load library … undefined symbol getTimestamp).
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

#include "turn_in_place_py.hpp"

namespace rob_box
{

namespace
{
// Клетка global/local costmap. Дальше неё с начала цели — уже прогресс, не ноль.
constexpr double kNoProgressM = 0.05;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class WouldSpinOnStall : public BT::ConditionNode
{
public:
  WouldSpinOnStall(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    node_->get_parameter_or("global_frame", global_frame_, std::string("map"));
    node_->get_parameter_or("robot_base_frame", base_frame_, std::string("base_footprint"));
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {scan_ = msg;});
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<double>("clearance_m", 0.10, "зазор «нос в стене», м — spin_stop_m защиты"),
      BT::OutputPort<double>("turn_spin_dist", "угол Spin, рад; тот же ключ доски, что spin_dist"),
    };
  }

  void halt() override
  {
    start_valid_ = false;
    BT::ConditionNode::halt();
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path) || path.poses.empty() || !scan_ || tf_ == nullptr) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (!current_pose(pose)) {
      return BT::NodeStatus::FAILURE;
    }
    if (!start_valid_) {
      start_ = pose;
      start_valid_ = true;
    }
    const double dx = pose.pose.position.x - start_.pose.position.x;
    const double dy = pose.pose.position.y - start_.pose.position.y;
    if (std::hypot(dx, dy) > kNoProgressM) {
      return BT::NodeStatus::FAILURE;
    }
    std::vector<std::pair<double, double>> points;
    if (!points_in_base(points)) {
      return BT::NodeStatus::FAILURE;
    }
    std::vector<std::pair<double, double>> way;
    if (!path_in_base(path, way)) {
      return BT::NodeStatus::FAILURE;
    }
    const double clearance = getInput<double>("clearance_m").value_or(0.10);
    double spin = 0.0;
    if (!free_heading_spin_rad(points, way, clearance, spin)) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("turn_spin_dist", spin);
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool current_pose(geometry_msgs::msg::PoseStamped & pose) const
  {
    geometry_msgs::msg::PoseStamped base;
    base.header.frame_id = base_frame_;
    base.header.stamp = scan_->header.stamp;
    base.pose.orientation.w = 1.0;
    try {
      pose = tf_->transform(base, global_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      return false;
    }
    return true;
  }

  bool points_in_base(std::vector<std::pair<double, double>> & points) const
  {
    double mx = 0.0;
    double my = 0.0;
    double myaw = 0.0;
    if (scan_->header.frame_id != base_frame_) {
      geometry_msgs::msg::PoseStamped laser;
      laser.header = scan_->header;
      laser.pose.orientation.w = 1.0;
      geometry_msgs::msg::PoseStamped in_base;
      try {
        in_base = tf_->transform(laser, base_frame_, tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException &) {
        return false;
      }
      mx = in_base.pose.position.x;
      my = in_base.pose.position.y;
      myaw = yaw_of(in_base.pose.orientation);
    }
    const double c = std::cos(myaw);
    const double s = std::sin(myaw);
    points.reserve(scan_->ranges.size());
    double angle = scan_->angle_min;
    for (float range : scan_->ranges) {
      if (std::isfinite(range) && range >= scan_->range_min && range <= scan_->range_max) {
        const double lx = range * std::cos(angle);
        const double ly = range * std::sin(angle);
        points.emplace_back(mx + c * lx - s * ly, my + s * lx + c * ly);
      }
      angle += scan_->angle_increment;
    }
    return true;
  }

  bool path_in_base(
    const nav_msgs::msg::Path & path, std::vector<std::pair<double, double>> & way) const
  {
    way.reserve(path.poses.size());
    for (const auto & stamped : path.poses) {
      geometry_msgs::msg::PoseStamped in = stamped;
      if (in.header.frame_id.empty()) {
        in.header.frame_id = path.header.frame_id;
      }
      if (in.header.frame_id != base_frame_) {
        try {
          in = tf_->transform(in, base_frame_, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException &) {
          return false;
        }
      }
      way.emplace_back(in.pose.position.x, in.pose.position.y);
    }
    return !way.empty();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  geometry_msgs::msg::PoseStamped start_;
  bool start_valid_{false};
  std::string global_frame_{"map"};
  std::string base_frame_{"base_footprint"};
};

}  // namespace rob_box

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rob_box::WouldSpinOnStall>("WouldSpinOnStall");
}
