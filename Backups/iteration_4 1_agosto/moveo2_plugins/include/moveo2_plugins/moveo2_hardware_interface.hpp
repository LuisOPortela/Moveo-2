#ifndef MOVEO2_PLUGINS__MOVEO2_PLUGINS_HPP_
#define MOVEO2_PLUGINS__MOVEO2_PLUGINS_HPP_

#include "moveo2_plugins/visibility_control.h"
#include "moveo2_plugins/moveo2_serial_port.hpp"
#include "serial/serial.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


using hardware_interface::return_type;

namespace moveo2_plugins
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class MOVEO2_PLUGINS_PUBLIC Moveo2HardwareInterface : public hardware_interface::SystemInterface
  {
    public:

      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
      
    protected:
      /// The size of this vector is (standard_interfaces_.size() x nr_joints)
      std::vector<double> joint_velocities_command_;
      std::vector<double> joint_position_state_;
      std::vector<double> joint_velocities_state_;
      //std::vector<double> ft_states_;
      //std::vector<double> ft_command_;

      std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
        {"position", {}}, {"velocity", {}}};

      MoveoSerialPort serial_port_;


  };

}  // namespace moveo2_plugins

#endif  // MOVEO2_PLUGINS__MOVEO2_PLUGINS_HPP_
