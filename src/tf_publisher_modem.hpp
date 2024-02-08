#pragma once
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>

namespace acoustic_simulator {
class TfPublisherModem : public rclcpp::Node {
 public:
  explicit TfPublisherModem(rclcpp::NodeOptions const &_options);

 private:
  struct Pose {
    double x{-0.1};
    double y{0.0};
    double z{0.0};
    double qw{0.7071068};
    double qx{0.0};
    double qy{0.7071068};
    double qz{0.0};
  };
  struct Params {
    bool has_modem{false};
    Pose modem;
  };
  void DeclareParameters();
  void DeclareModemParameters();
  void DeclareModemPoseParameters(std::string _modem_name, Pose &_pose_param);
  void BroadCastStatic();

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  Params params_;
};
}  // namespace acoustic_simulator
