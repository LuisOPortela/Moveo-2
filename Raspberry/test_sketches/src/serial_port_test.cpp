#include <serial/serial.h>
#include <iostream>
#include <limits>


int main()
{
    serial::Serial serial_conn_;
    serial_conn_.setPort("/dev/ttyACM0");
    serial_conn_.setBaudrate(9600);
    serial::Timeout tt = serial::Timeout::simpleTimeout(1000);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();

    while(1){
        std::string value;
        std::cout << "Please enter an unsigned integer: ";
        std::cin >> value;
        // Send a message
         serial_conn_.write(value+"\n");

        // Receive a message

    }
    return 0;
}
