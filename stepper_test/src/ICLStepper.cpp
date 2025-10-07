#include "stepper_test/ICLStepper.h"

#include <iostream>
#include <unistd.h>
#include <cerrno>
#include <modbus/modbus.h>

ICLStepper::ICLStepper(int slave_id, modbus_t* ctx)
    : slave_id_(slave_id),
      ctx_(ctx),
      current_position_(0),
      delay_us_(10) {}

int ICLStepper::initialize() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // Enable motor
    if (modbus_write_register(ctx_, 0x000F, 1) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to enable motor: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    // Example: set a target speed (100)
    if (modbus_write_register(ctx_, 0x01E1, 100) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set target speed: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    return 0;
}

int ICLStepper::set_position(int position, int velocity_rpm, int acc, int dec) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // 1) Prepare PR0 configuration block
    uint16_t pos_high = static_cast<uint16_t>((position >> 16) & 0xFFFF);
    uint16_t pos_low  = static_cast<uint16_t>(position & 0xFFFF);
    uint16_t pr0_registers[] = {
        0x0001,     // mode: absolute positioning
        pos_high,
        pos_low,
        static_cast<uint16_t>(velocity_rpm),
        static_cast<uint16_t>(acc),
        static_cast<uint16_t>(dec)
    };
    if (modbus_write_registers(ctx_, 0x6200, sizeof(pr0_registers) / sizeof(pr0_registers[0]), pr0_registers) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to configure PR0 registers: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Configured PR0 registers (mode, position, velocity, acc/dec)" << std::endl;
    usleep(delay_us_);

    // Trigger PR0 motion
    if (modbus_write_register(ctx_, 0x6002, 0x0010) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to trigger PR0 motion: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    std::cout << "Triggering PR0 motion" << std::endl;

    return 0;
}

int ICLStepper::get_position() const {
    // Placeholder: in a real implementation, read back from the drive’s position registers.
    return current_position_;
}

int ICLStepper::jog(bool clockwise) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    int command = clockwise ? 0x4001 : 0x4002;
    if (modbus_write_register(ctx_, 0x1801, command) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to jog motor: " << modbus_strerror(errno) << std::endl;
        return -1;
    }
    // Wait delay_us microseconds
    usleep(delay_us_);

    return 0;
}

int ICLStepper::set_jog_acceleration(int acc) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // set jog acceleration/deceleration
    if (modbus_write_register(ctx_, 0x01E7, acc) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set jog acc/dec: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    return 0;
}

int ICLStepper::set_jog_velocity(int velocity_rpm) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    if (modbus_write_register(ctx_, 0x01E1, velocity_rpm) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set target jog velocity: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);
    return 0;
}

int ICLStepper::set_slave_id(int slave_id) {
    slave_id_ = slave_id;
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);
    return 0;
}

/*
bit0: Fault
bit1: Enable
bit2: Running
bit4: Command Completed
bit5: Path Completed
bit6: Homing Completed
*/
uint16_t ICLStepper::read_motion_status() {
    uint16_t status;
    if (modbus_read_registers(ctx_, 0x1003, 1, &status) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to read motion status: "
                  << modbus_strerror(errno) << std::endl;
        return 0;
    }
    usleep(delay_us_);

    std::cout   << "Fault: " << (status & 1) << "\n"
                << "Enable: " << ((status >> 1) & 1) << "\n"
                << "In Motion: " << ((status >> 2) & 1) << "\n"
                << "Command Done: " << ((status >> 4) & 1) << "\n"
                << "Path Done: " << ((status >> 5) & 1) << "\n"
                << "Homing Done: " << ((status >> 6) & 1) << "\n";

    return static_cast<uint16_t>(status & 0xFF);
}
