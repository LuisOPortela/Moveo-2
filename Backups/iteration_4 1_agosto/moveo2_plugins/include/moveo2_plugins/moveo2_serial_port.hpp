#ifndef MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_
#define MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_


#include "serial/serial.h"
#include <cstring>

class MoveoSerialPort
{
    public:
    MoveoSerialPort()
    {  }

    MoveoSerialPort(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {  }

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    void sendEmptyMsg();
    
    void readEncoderValues(int &val_1, int &val_2);
    void setMotorValues(int val_1, int val_2);
    void setPidValues(float k_p, float k_d, float k_i, float k_o);

    bool connected() const { return serial_conn_.isOpen(); }

    void sendMsg(const std::string &msg_to_send);

    std::string read();
    
    private:
    serial::Serial serial_conn_;  ///< Underlying serial connection 
};




#endif // MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_