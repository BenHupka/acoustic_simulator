#include "tf_publisher_modem.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace acoustic_simulator {

TfPublisherModem::TfPublisherModem(rclcpp::NodeOptions const &_options)
    : Node("tf_publisher_modem", _options) {
  static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  DeclareParameters();
  BroadCastStatic();
}

void TfPublisherModem::DeclareParameters() {
  DeclareModemParameters();
}

void TfPublisherModem::DeclareModemPoseParameters(std::string _modem_name,
                                                 Pose &_pose_param) {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = _modem_name + ".x";
  descr_text = "Modem offset in x direction relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.x = declare_parameter(name, _pose_param.x, descr);

  name = _modem_name + ".y";
  descr_text = "Modem offset in y direction relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.y = declare_parameter(name, _pose_param.y, descr);

  name = _modem_name + ".z";
  descr_text = "Modem offset in z direction relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.z = declare_parameter(name, _pose_param.z, descr);

  name = _modem_name + ".qw";
  descr_text =
      "Quaternion component of the modem orientation relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.qw = declare_parameter(name, _pose_param.qw, descr);

  name = _modem_name + ".qx";
  descr_text =
      "Quaternion component of the modem orientation relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.qx = declare_parameter(name, _pose_param.qx, descr);

  name = _modem_name + ".qy";
  descr_text =
      "Quaternion component of the modem orientation relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.qy = declare_parameter(name, _pose_param.qy, descr);

  name = _modem_name + ".qz";
  descr_text =
      "Quaternion component of the modem orientation relative to base_link";
  descr = hippo_common::param_utils::Description(descr_text, true);
  _pose_param.qz = declare_parameter(name, _pose_param.qz, descr);
}

void TfPublisherModem::DeclareModemParameters() {
  std::string name;
  rcl_interfaces::msg::ParameterDescriptor descr;
  std::string descr_text;

  name = "has_modem";
  descr_text = "If vehicle has acoustic modem";
  descr = hippo_common::param_utils::Description(descr_text, true);
  params_.has_modem = declare_parameter(name, false, descr);
  if (params_.has_modem) {
    DeclareModemPoseParameters("modem", params_.modem);
  }
}

void TfPublisherModem::BroadCastStatic() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  // transformation between baselink and modem link
  if (params_.has_modem) {
    RCLCPP_INFO_STREAM(get_logger(), "Publishing modem transformation.");
    geometry_msgs::msg::TransformStamped t;
    t.transform.translation.x = params_.modem.x;
    t.transform.translation.y = params_.modem.y;
    t.transform.translation.z = params_.modem.z;
    t.transform.rotation.w = params_.modem.qw;
    t.transform.rotation.x = params_.modem.qx;
    t.transform.rotation.y = params_.modem.qy;
    t.transform.rotation.z = params_.modem.qz;
    t.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
    t.child_frame_id = hippo_common::tf2_utils::frame_id::ModemLink(this);
    transforms.push_back(t);
  }
  static_broadcaster_->sendTransform(transforms);
}
}  // namespace acoustic_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acoustic_simulator::TfPublisherModem)
