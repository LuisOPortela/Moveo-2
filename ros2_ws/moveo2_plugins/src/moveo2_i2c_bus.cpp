#include "moveo2_plugins/moveo2_i2c_bus.hpp"
#include "rclcpp/rclcpp.hpp"

#include <linux/i2c-dev.h>  //i2c_slave
#include <fcntl.h>          //open
#include <sys/ioctl.h>      //ioctl
//#include <unistd.h>         //write 
#include <cmath>            // M_PI
/*
using SysWriteFunc = ssize_t(*)(int, const void*, size_t);
const SysWriteFunc i2c_write = ::write;
*/


void Moveo2I2C::setup(const char *bus)
{  

  if((i2c_conn_ = open(bus, O_RDWR)) < 0)
  {
    throw std::runtime_error("Error opening file/device");
  }
  
}

void Moveo2I2C::set_device(const int adress)
{

  if(ioctl(i2c_conn_, I2C_SLAVE, adress)==-1)
  {
    throw std::runtime_error("Setting the adress failed " + std::string(strerror(errno)));
  }

}

double Moveo2I2C::read_magnitude()
{

    const char M[1]={0xFE};       //Define that we want magnitude values from the encoder
    
    if(::write(i2c_conn_, M, 1)==-1)
    {
        throw std::runtime_error("Error defining the desired values");    
    }
    
    char D[2]={0};
    if(::read(i2c_conn_,D,2)!=2)
    {
        throw std::runtime_error("Error reading sensor");    
    }

    int y= D[0];
    int x= D[1];

    double joint_position = ((x<<6) +(y & 0x3F))*2*M_PI/(1 << 14)-M_PI;   

    return joint_position;
}

double Moveo2I2C::read_sensor(const int adress)
{
    set_device(adress);
    return read_magnitude();
}