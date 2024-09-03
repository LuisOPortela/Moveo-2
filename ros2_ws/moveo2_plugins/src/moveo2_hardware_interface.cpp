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

  //*********************************************************************
  //*                  Parameters (from ros2_control.xacro)
  
  try {
    cfg_.i2c_bus = info_.hardware_parameters.at("i2c_bus");
    cfg_.serial_device = info_.hardware_parameters.at("serial_device");
    cfg_.baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    cfg_.timeout = std::stoi(info_.hardware_parameters.at("timeout"));

    for (const auto& joint : info_.joints) 
    {
        moveo2_joints_.emplace_back(Joint{
            joint.name,
            std::stoi(joint.parameters.at("steps_per_revolution")),
            std::stoi(joint.parameters.at("encoder_i2c_adress"))
        });
    }
  } 
  catch (const std::out_of_range& e) 
  {
    RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%sOn_init -> Parameter missing ERROR: %s%s",RED.c_str(),e.what(),RESET.c_str());
    return CallbackReturn::ERROR;
  } 
  catch (const std::invalid_argument& e) 
  {
    RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%sOn_init -> Invalid parameter ERROR: %s%s",RED.c_str(),e.what(),RESET.c_str());
    return CallbackReturn::ERROR;
  }

  //**********************************************************************
  //*                             Serial

  try
  {
    serial_conn_.setup(cfg_.serial_device, cfg_.baud_rate , cfg_.timeout );  
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%sOn_init -> Serial Port ERROR (Usually device not connected): %s%s",RED.c_str(),e.what(),RESET.c_str());
    return CallbackReturn::ERROR;
  }
  
   
  //***********************************************************************
  //*                               I2C
  
  try
  {
    i2c_conn_.setup(cfg_.i2c_bus.c_str());  
  }
  catch(const std::exception& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%sOn_init -> I2C Bus ERROR:%s",RED.c_str(),RESET.c_str());
    return CallbackReturn::ERROR;
  } 
  
  //********************************************************************** 

  RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%sOn_init -> Success, Finished Setup.%s",GREEN.c_str(),RESET.c_str());
  return CallbackReturn::SUCCESS;
}


//################################################################################
//#################          export_state_interfaces            ##################
//################################################################################
// This assumes all joints have the same state interfaces

std::vector<hardware_interface::StateInterface> Moveo2HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto&joint : moveo2_joints_)
  {
    state_interfaces.emplace_back(joint.name, "position", &joint.position_state);
    state_interfaces.emplace_back(joint.name, "velocity", &joint.velocities_state);
  }

  return state_interfaces;
}


//################################################################################
//#################         export_command_interfaces           ##################
//################################################################################
// This assumes all joints have the same command interfaces

std::vector<hardware_interface::CommandInterface> Moveo2HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : moveo2_joints_)
  {
    command_interfaces.emplace_back(joint.name, "velocity", &joint.velocities_command);
  }

  return command_interfaces;
}


//################################################################################
//#################                   read                      ##################
//################################################################################


return_type Moveo2HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) 
{
  for (auto & joint : moveo2_joints_)
  {
    const auto joint_position_previous = joint.position_state;
    double joint_position;
  
    try
    {
      joint_position = i2c_conn_.read_sensor(joint.encoder_i2c_adress);
      //RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%s Read -> Sensor value %f:%s",GREEN.c_str(),joint_position,RESET.c_str());
    }
    catch (const std::exception& e)
    {
       RCLCPP_INFO(rclcpp::get_logger("Moveo2HardwareInterface"),"%s Read -> I2C Bus error:%s%s",RED.c_str(),e.what(),RESET.c_str());
    }
  
    const double delta_seconds = period.seconds();
    joint.position_state = joint_position;
    joint.velocities_state = (joint_position- joint_position_previous)/delta_seconds;
    
    
  /*    
    for (auto & joint : moveo2_joints_)
    {
      joint.velocities_state = joint.velocities_command;
      joint.position_state += joint.velocities_command * period.seconds();
    }
  */  
  }

  return return_type::OK;
}

//################################################################################
//#################                   write                     ##################
//################################################################################


return_type Moveo2HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  //*Send velocities through the serial port
  // DISTINGUI ENTRE OS JOINTS AO MANDAR VALORES
  for (auto & joint : moveo2_joints_)
  {
    serial_conn_.sendMsg(std::to_string(joint.velocities_command) + 's');
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing joint velocitie command : %f", joint.velocities_command);
  }
  
  return return_type::OK;
}

}  // namespace moveo2_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moveo2_plugins::Moveo2HardwareInterface, hardware_interface::SystemInterface)
