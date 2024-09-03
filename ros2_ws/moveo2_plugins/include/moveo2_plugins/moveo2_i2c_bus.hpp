#ifndef MOVEO2_PLUGINS__MOVEO2_I2C_HPP_
#define MOVEO2_PLUGINS__MOVEO2_I2C_HPP_

class Moveo2I2C
{
    public:
    Moveo2I2C()
    {  }

    void setup(const char *bus);
    void set_device(const int adress);
    double read_magnitude();
    double read_sensor(const int adress);

    private:
    int i2c_conn_;

};

#endif // MOVEO2_PLUGINS__MOVEO2_SERIAL_HPP_