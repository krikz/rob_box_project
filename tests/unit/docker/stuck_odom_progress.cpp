#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Only ROS transport, clock and the costmap are adapted. The observation callback,
// command window and source selection below come from the patched ControllerServer.
struct Stamp { int sec; unsigned nanosec; };
struct Header { std::string frame_id; Stamp stamp; };
struct Point { double x{}, y{}, z{}; };
struct Quaternion { double x{}, y{}, z{}, w{1}; };
struct Pose { Point position; Quaternion orientation; };
struct CovariantPose { Pose pose; std::array<double, 36> covariance{}; };
namespace nav_msgs::msg {
struct Odometry {
  using ConstSharedPtr = std::shared_ptr<const Odometry>;
  Header header;
  std::string child_frame_id;
  CovariantPose pose;
};
}
namespace geometry_msgs::msg { struct PoseStamped { Pose pose; }; }
namespace tf2 { double getYaw(const Quaternion &) { return 0.; } }
namespace nav2_util {
// Inputs to this test have valid ROS message shapes. Tracking loss uses RTAB's covariance flag.
bool validateMsg(const Header &) { return true; }
bool validateMsg(const CovariantPose &) { return true; }
}
struct ControllerServer {
  struct StuckSample { double t, x, y; bool held; };
  struct Clock { double t; double seconds() const { return t; } };
  struct Costmap { std::string getBaseFrameID() const { return "base_footprint"; } };
  struct Twist { Point linear, angular; };
  Costmap map;
  Costmap *costmap_ros_{&map};
  std::string stuck_progress_frame_{"icp_odom"};
  std::mutex stuck_odom_mutex_;
  std::vector<StuckSample> stuck_samples_, stuck_odom_samples_;
  double time{}, stuck_cmd_since_{-1}, stuck_last_course_{}, stuck_last_course_time_{-1};
  Twist last_cmd_vel_;
  bool have_last_cmd_{true};
  Clock now() const { return {time}; }
  bool stuckAgainstCourseBlocked(double vx, double x, double y, double) const {
    // Wall in wheel odom. The ICP frame has a different origin.
    return vx > 0.03 && x < 1. && y == 0.;
  }
  #include "motion_declarations.inc"
  void stuckAgainstReset();
  bool stuckAgainstObstacle(const geometry_msgs::msg::PoseStamped &);
};
#include "motion_methods.inc"

int main() {
  for (double icp_distance : {0., .184913}) {
    ControllerServer controller;
    controller.last_cmd_vel_.linear.x = .123;
    for (int tick = 0; tick <= 82; ++tick) {
      const int millis = 300000 + tick * 50;
      controller.time = millis / 1000.;
      if (tick % 2 == 0) {
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom->header = {"icp_odom", {millis / 1000, unsigned(millis % 1000) * 1000000}};
        odom->child_frame_id = "base_footprint";
        odom->pose.pose.position.x = 9. + icp_distance * tick / 40.;
        if (tick == 42) {
          odom->pose.covariance[0] = 9999.;
          controller.stuckAgainstOdom(odom);
          odom->pose.covariance[0] = 0.;
        }
        controller.stuckAgainstOdom(odom);
      }
      geometry_msgs::msg::PoseStamped wheel;
      wheel.pose.position.x = .031809 * tick / 40.;
      const bool stuck = controller.stuckAgainstObstacle(wheel);
      const bool full_window = tick == 40 || tick == 41 || tick == 82;
      assert(stuck == (icp_distance == 0. && full_window));
    }
  }
}
