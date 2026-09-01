#ifndef MOTOR_SERIAL_HPP
#define MOTOR_SERIAL_HPP

#include <string>
#include <libserial/SerialPort.h>

class MotorSerial {
public:
    MotorSerial();

    bool initialize(const std::string &port = "/dev/ttyACM0",
                    unsigned long baud = 115200);

    bool is_open() const;

    void set_angle_radians(double radians);
    void send_custom(const std::string &cmd_str);

    void enable_motor();
    void disable_motor();

private:
    LibSerial::SerialPort serial_;
};

#endif // MOTOR_SERIAL_HPP
