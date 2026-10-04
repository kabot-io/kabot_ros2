#include <kabot_robot/motion_calibration.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <control_msgs/msg/multi_dof_state_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace kabot_robot
{
class CalibratedMotionController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    auto_declare<std::vector<std::string>>("joints", {"left_wheel_joint", "right_wheel_joint"});
    auto_declare<double>("wheel_radius", 0.016);
    auto_declare<double>("wheel_separation", 0.102);
    const MotionCalibration defaults;
    auto_declare<double>("calibration.low_speed", defaults.low_speed);
    auto_declare<double>("calibration.high_speed", defaults.high_speed);
    auto_declare<std::vector<double>>("calibration.low_effort", {defaults.low_effort[0], defaults.low_effort[1]});
    auto_declare<std::vector<double>>("calibration.high_effort", {defaults.high_effort[0], defaults.high_effort[1]});
    auto_declare<double>("calibration.ccw_rate", defaults.ccw_rate);
    auto_declare<double>("calibration.cw_rate", defaults.cw_rate);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      joints_ = get_node()->get_parameter("joints").as_string_array();
      radius_ = get_node()->get_parameter("wheel_radius").as_double();
      separation_ = get_node()->get_parameter("wheel_separation").as_double();
      const auto low = get_node()->get_parameter("calibration.low_effort").as_double_array();
      const auto high = get_node()->get_parameter("calibration.high_effort").as_double_array();
      if (joints_.size() != 2 || joints_[0] == joints_[1] || low.size() != 2 || high.size() != 2 ||
          !std::isfinite(radius_) || !std::isfinite(separation_) || radius_ <= 0 || separation_ <= 0) {
        throw std::invalid_argument("Expected two wheels and positive finite geometry");
      }
      MotionCalibration calibration;
      calibration.low_speed = get_node()->get_parameter("calibration.low_speed").as_double();
      calibration.high_speed = get_node()->get_parameter("calibration.high_speed").as_double();
      calibration.low_effort = {low[0], low[1]};
      calibration.high_effort = {high[0], high[1]};
      calibration.ccw_rate = get_node()->get_parameter("calibration.ccw_rate").as_double();
      calibration.cw_rate = get_node()->get_parameter("calibration.cw_rate").as_double();
      model_ = std::make_unique<MotionModel>(calibration);
      joint_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
        get_node()->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10));
      joint_message_.name = joints_;
      joint_message_.position.resize(2);
      joint_message_.velocity.resize(2);
      joint_message_.effort.resize(2);
      state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>>(
        get_node()->create_publisher<control_msgs::msg::MultiDOFStateStamped>("~/controller_state", 10));
      state_message_.dof_states.resize(2);
      for (std::size_t wheel = 0; wheel < 2; ++wheel) {
        state_message_.dof_states[wheel].name = joints_[wheel];
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", error.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {joints_[0] + "/effort", joints_[1] + "/effort"}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      bool found = false;
      for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
        if (command_interfaces_[index].get_name() == joints_[wheel] + "/effort") {
          command_index_[wheel] = index;
          found = true;
        }
      }
      if (!found) {
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    references_.fill(0);
    velocities_.fill(0);
    positions_.fill(0);
    return write_efforts({0, 0}) ? controller_interface::CallbackReturn::SUCCESS :
      controller_interface::CallbackReturn::ERROR;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    references_.fill(0);
    velocities_.fill(0);
    return write_efforts({0, 0}) ? controller_interface::CallbackReturn::SUCCESS :
      controller_interface::CallbackReturn::ERROR;
  }

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
        std::string(get_node()->get_name()) + "/" + joints_[wheel], "velocity", &references_[wheel]));
    }
    return interfaces;
  }

  std::vector<hardware_interface::StateInterface::SharedPtr> on_export_state_interfaces_list() override
  {
    std::vector<hardware_interface::StateInterface::SharedPtr> interfaces;
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
        std::string(get_node()->get_name()) + "/" + joints_[wheel], "velocity", &velocities_[wheel]));
    }
    return interfaces;
  }

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    references_.fill(0);
    return controller_interface::return_type::OK;
  }

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const auto output = model_->map(
      (references_[0] + references_[1]) * radius_ / 2,
      (references_[1] - references_[0]) * radius_ / separation_);
    if (!write_efforts(output.effort)) {
      (void)write_efforts({0, 0});
      velocities_.fill(0);
      return controller_interface::return_type::ERROR;
    }
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      const double direction = wheel == 0 ? -1.0 : 1.0;
      positions_[wheel] += velocities_[wheel] * period.seconds();
      velocities_[wheel] = (output.linear + direction * output.angular * separation_ / 2) / radius_;
      joint_message_.position[wheel] = positions_[wheel];
      joint_message_.velocity[wheel] = velocities_[wheel];
      joint_message_.effort[wheel] = output.effort[wheel];
      auto & state = state_message_.dof_states[wheel];
      state.reference = references_[wheel];
      state.feedback = velocities_[wheel];
      state.error = references_[wheel] - velocities_[wheel];
      state.output = output.effort[wheel];
    }
    joint_message_.header.stamp = time;
    state_message_.header.stamp = time;
    (void)joint_publisher_->try_publish(joint_message_);
    (void)state_publisher_->try_publish(state_message_);
    return controller_interface::return_type::OK;
  }

private:
  bool write_efforts(const std::array<double, 2> & efforts)
  {
    bool success = true;
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      success = command_interfaces_[command_index_[wheel]].set_value(efforts[wheel]) && success;
    }
    return success;
  }

  std::vector<std::string> joints_{"left_wheel_joint", "right_wheel_joint"};
  double radius_ = 0.016;
  double separation_ = 0.102;
  std::array<double, 2> references_{};
  std::array<double, 2> velocities_{};
  std::array<double, 2> positions_{};
  std::array<std::size_t, 2> command_index_{};
  std::unique_ptr<MotionModel> model_;
  sensor_msgs::msg::JointState joint_message_;
  control_msgs::msg::MultiDOFStateStamped state_message_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> joint_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::MultiDOFStateStamped>> state_publisher_;
};
}

PLUGINLIB_EXPORT_CLASS(kabot_robot::CalibratedMotionController, controller_interface::ChainableControllerInterface)