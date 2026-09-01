#include "stepper_test/MotorSerial.hpp"
#include <iostream>
#include <stdexcept>

using LibSerial::SerialPort;
using LibSerial::BaudRate;

// Helper: map numeric baud to LibSerial::BaudRate
static BaudRate to_baud_rate(unsigned long baud)
{
    switch (baud) {
        case 9600:    return BaudRate::BAUD_9600;
        case 19200:   return BaudRate::BAUD_19200;
        case 38400:   return BaudRate::BAUD_38400;
        case 57600:   return BaudRate::BAUD_57600;
        case 115200:  return BaudRate::BAUD_115200;
        default:
            // Fallback: LibSerial has a default; you can tweak this.
            std::cerr << "Warning: unsupported baud " << baud
                      << ", falling back to 115200.\n";
            return BaudRate::BAUD_115200;
    }
}

MotorSerial::MotorSerial() = default;

bool MotorSerial::initialize(const std::string &port, unsigned long baud)
{
    try {
        // Open the port
        serial_.Open(port);

        // Basic configuration – adjust if you need other settings.
        serial_.SetBaudRate(to_baud_rate(baud));

        // If your libserial version supports these, you can uncomment:
        // serial_.SetCharSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        // serial_.SetParity(LibSerial::Parity::PARITY_NONE);
        // serial_.SetNumOfStopBits(LibSerial::StopBits::STOP_BITS_1);
        // serial_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        std::cout << "Serial connection opened on " << port
                  << " at " << baud << " baud\n";
        return true;
    }
    catch (const std::exception &e) {
        std::cerr << "Serial open/config failed: " << e.what() << "\n";
        return false;
    }
}

bool MotorSerial::is_open() const
{
    return serial_.IsOpen();
}

void MotorSerial::set_angle_radians(double radians)
{
    if (!serial_.IsOpen()) {
        return;
    }

    double motor_angle = radians * 100.0;
    std::string cmd = "M" + std::to_string(motor_angle) + "\n";

    try {
        serial_.Write(cmd);
    }
    catch (const std::exception &e) {
        std::cerr << "Serial write failed (set_angle_radians): "
                  << e.what() << "\n";
    }
}

void MotorSerial::send_custom(const std::string &cmd_str)
{
    if (!serial_.IsOpen()) {
        return;
    }

    try {
        serial_.Write(cmd_str + "\n");
    }
    catch (const std::exception &e) {
        std::cerr << "Serial write failed (send_custom): "
                  << e.what() << "\n";
    }
}

void MotorSerial::enable_motor()
{
    if (!serial_.IsOpen()) {
        return;
    }

    try {
        serial_.Write(std::string("E1\n"));
    }
    catch (const std::exception &e) {
        std::cerr << "Serial write failed (enable_motor): "
                  << e.what() << "\n";
    }
}

void MotorSerial::disable_motor()
{
    if (!serial_.IsOpen()) {
        return;
    }

    try {
        serial_.Write(std::string("E0\n"));
    }
    catch (const std::exception &e) {
        std::cerr << "Serial write failed (disable_motor): "
                  << e.what() << "\n";
    }
}
