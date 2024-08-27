#include "moveo2_plugins/moveo2_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moveo2_plugins
{



/*Writing an hardware interface:
Implement on_init method. Here, you should initialize all member variables and process the parameters from the info argument. 
In the first line usually the parents on_init is called to process standard values, like name. 
This is done using: hardware_interface::(Actuator|Sensor|System)Interface::on_init(info). 
If all required parameters are set and valid and everything works fine return CallbackReturn::SUCCESS or return CallbackReturn::ERROR otherwise.

Ros2_control example 7:
The on_init method is called once during ros2_control initialization if the 'Moveo2HardwareInterface' was specified in the URDF. 
In this method, communication between the robot hardware needs to be setup and memory dynamic should be allocated. 

Instead, vectors will be initialized that represent the state all the hardware, e.g. a vector of doubles describing joint angles, etc.
*/


//################################################################################
//#################                   on_init                   ##################
//################################################################################

hardware_interface::CallbackReturn Moveo2HardwareInterface::on_init(                    // ON INIT
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }


  //*Receive params defined in the ros2_control.xacro!!!!!!!!!!!!!!


  //#######

  //***********************************************************************


  //*This init function must initialize the serial comms with the arduino

  serial_port_.setup("/dev/ttyACM0", 9600 , 1000 );  
  
  //***********************************************************************

  // robot has 6 joints and 2 interfaces
  joint_position_state_.assign(1, 0);
  joint_velocities_state_.assign(1, 0);
  joint_velocities_command_.assign(1, 0);

  // force sensor has 6 readings
  //ft_states_.assign(6, 0);
  //ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

//################################################################################
//#################          export_state_interfaces            ##################
//################################################################################

std::vector<hardware_interface::StateInterface> Moveo2HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_state_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_state_[ind++]);
  }

  //state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  //state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  //state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  //state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  //state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  //state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

//################################################################################
//#################         export_command_interfaces           ##################
//################################################################################

std::vector<hardware_interface::CommandInterface> Moveo2HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;


  int ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }


  return command_interfaces;
}

//################################################################################
//#################                   read                      ##################
//################################################################################


return_type Moveo2HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) 
{
    //*This read function must receive the values from the encoder trough the i2c door!!!!!

  //std::string response = serial_port_.read();
  //RCLCPP_INFO(rclcpp::get_logger("Writing joint velocitie command : %f"), response.c_str());

    //*Calculate the velocity with the position!!!!
  //for i in joints
  auto i= 0;

  auto joint_position_previous = joint_position_state_[i];
  auto joint_position = 0;
  
  double delta_seconds= period.seconds();
  joint_position_state_[i]= joint_position;
  joint_velocities_state_[i]= (joint_position_state_[i]- joint_position_previous)/delta_seconds;
  
  
  
  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_state_[i] = joint_velocities_command_[i];
    joint_position_state_[i] += joint_velocities_command_[i] * period.seconds();
  }

  return return_type::OK;
}

//################################################################################
//#################                   write                     ##################
//################################################################################


return_type Moveo2HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  //*Send velocities through the serial port
  serial_port_.sendMsg(std::to_string(joint_velocities_command_[0]) + 's');
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using write");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing joint velocitie command : %f", joint_velocities_command_[0]);

  
  return return_type::OK;
}

}  // namespace moveo2_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moveo2_plugins::Moveo2HardwareInterface, hardware_interface::SystemInterface)
