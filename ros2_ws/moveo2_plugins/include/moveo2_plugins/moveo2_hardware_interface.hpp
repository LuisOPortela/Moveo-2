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

  struct Config
  {
    std::string i2c_bus = "/dev/i2c-1";
    std::string serial_device;
    int baud_rate = 9600;
    int timeout = 1000;
  };

  struct Joint
  {
    std::string name;
    int steps_per_revolution;
    int encoder_i2c_adress;
    double velocities_command=0;
    double position_state=0;
    double velocities_state=0;


  };

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
      Config cfg_;
      std::vector<Joint> moveo2_joints_;
      Moveo2SerialPort serial_port_;
      int i2c_bus_;

  };

}  // namespace moveo2_plugins

#endif  // MOVEO2_PLUGINS__MOVEO2_PLUGINS_HPP_
