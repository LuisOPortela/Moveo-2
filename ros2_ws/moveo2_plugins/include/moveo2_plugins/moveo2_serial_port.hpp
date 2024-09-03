#ifndef MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_
#define MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_


#include "serial/serial.h"
#include <cstring>

class Moveo2SerialPort
{
    public:
    Moveo2SerialPort()
    {  }

    Moveo2SerialPort(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {  }

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendEmptyMsg();
    void sendMsg(const std::string &msg_to_send);
    
    private:
    serial::Serial serial_conn_;  ///< Underlying serial connection 
};




#endif // MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_