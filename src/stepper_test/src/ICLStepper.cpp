#include "stepper_test/ICLStepper.h"

#include <iostream>
#include <unistd.h>
#include <cerrno>
#include <modbus/modbus.h>
#include <cmath>

ICLStepper::ICLStepper(int slave_id, modbus_t* ctx, int pulses_per_revolution, int gear_ratio)
    : slave_id_(slave_id),
      ctx_(ctx),
      pulses_per_revolution_(pulses_per_revolution),
      gear_ratio_(gear_ratio),
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

    return 0;
}

int ICLStepper::disable_motor() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // Disable motor
    if (modbus_write_register(ctx_, 0x000F, 0) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to disable motor: "
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
    // std::cout << "Configured PR0 registers (mode, position, velocity, acc/dec)" << std::endl;
    usleep(delay_us_);

    // Trigger PR0 motion
    if (modbus_write_register(ctx_, 0x6002, 0x0010) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to trigger PR0 motion: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    // std::cout << "Triggering PR0 motion" << std::endl;

    return 0;
}

int ICLStepper::set_position_radians(double position_radians, double radians_per_second) {
    double revolutions = position_radians / (2.0 * M_PI);
    int target_position = static_cast<int>(revolutions * pulses_per_revolution_ * gear_ratio_);
    int velocity_rpm = static_cast<int>(gear_ratio_*(radians_per_second * 60.0) / (2.0 * M_PI));
    return set_position(target_position, velocity_rpm, 2000, 2000);
}

int32_t ICLStepper::read_position() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    uint16_t pos_high;
    uint16_t pos_low;

    if (modbus_read_registers(ctx_, 0x602C, 1, &pos_high) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to read position high bits: "
                  << modbus_strerror(errno) << std::endl;
        return INT32_MIN;
    }
    usleep(delay_us_);

    if (modbus_read_registers(ctx_, 0x602D, 1, &pos_low) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to read position low bits: "
                  << modbus_strerror(errno) << std::endl;
        return INT32_MIN;
    }
    usleep(delay_us_);

    int32_t high = static_cast<int32_t>(static_cast<int16_t>(pos_high));
    return (high << 16) | static_cast<int32_t>(pos_low);
}

double ICLStepper::get_position_radians(){
    int32_t position = read_position();
    if (position == INT32_MIN){
        return 0.0; // Error reading position
    }
    double revolutions = static_cast<double>(position) / (static_cast<double>(pulses_per_revolution_) * gear_ratio_);
    return revolutions * 2.0 * M_PI;
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

    // std::cout   << "Fault: " << (status & 1) << "\n"
    //             << "Enable: " << ((status >> 1) & 1) << "\n"
    //             << "In Motion: " << ((status >> 2) & 1) << "\n"
    //             << "Command Done: " << ((status >> 4) & 1) << "\n"
    //             << "Path Done: " << ((status >> 5) & 1) << "\n"
    //             << "Homing Done: " << ((status >> 6) & 1) << "\n";

    return static_cast<uint16_t>(status & 0xFF);
}

/*
0x600A: Homing mode
    bit0: homing direction = 0: CCW, 1: CW
    bit1: move to specificed point after homing = 0: no, 1: yes
    bit2: homing type = 0: limit switch signal, 1: home switch signal
0x600B: Home Switch position high bits
0x600C: Home Switch position low bits
0x600D: Homing stop position high bits
0x600E: Homing stop position low bits
0x600F: Homing high velocity rpm
0x6010: Homing low velocity rpm
0x6011: Homing acceleration ms/1000rpm
0x6012: Homing decceleration ms/1000rpm
*/
int ICLStepper::home(double home_switch_position, double position_after_homing_radians, bool clockwise, double radians_per_second) {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    int32_t position_after_homing = static_cast<int32_t>( (position_after_homing_radians / (2.0 * M_PI)) * pulses_per_revolution_ * gear_ratio_ );
    int32_t home_switch_pulses = static_cast<int32_t>( (home_switch_position / (2.0 * M_PI)) * pulses_per_revolution_ * gear_ratio_ );
    
    uint16_t pos_high_stop = static_cast<uint16_t>((position_after_homing >> 16) & 0xFFFF);
    uint16_t pos_low_stop  = static_cast<uint16_t>(position_after_homing & 0xFFFF);

    uint16_t pos_high_switch = static_cast<uint16_t>((home_switch_pulses >> 16) & 0xFFFF);
    uint16_t pos_low_switch  = static_cast<uint16_t>(home_switch_pulses & 0xFFFF);

    // Convert radians per second to RPM
    uint high_velocity_rpm = static_cast<int>(gear_ratio_ * (radians_per_second * 60.0) / (2.0 * M_PI));
    uint low_velocity_rpm = high_velocity_rpm / 2; // low velocity is half of high velocity

    // Configure homing parameters
    uint16_t homing_params[] = {
        (clockwise ? 0b111 : 0b110), // Homing mode
        pos_high_switch, // Home Switch position high bits
        pos_low_switch, // Home Switch position low bits
        pos_high_stop, // Homing stop position high bits
        pos_low_stop,  // Homing stop position low bits
        high_velocity_rpm,    // Homing high velocity
        low_velocity_rpm,      // Homing low velocity
        2000,     // Homing acceleration
        2000      // Homing deceleration
    };
    if (modbus_write_registers(ctx_, 0x600A, sizeof(homing_params) / sizeof(homing_params[0]), homing_params) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to configure homing parameters: "
                    << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    // Trigger homing
    if (modbus_write_register(ctx_, 0x6002, 0x0020) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to trigger homing: "
                    << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    return 0;
}

int ICLStepper::configure_io_for_homing() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // Configure digital input 2 as home switch
    if (modbus_write_register(ctx_, 0x0147, 0x0027) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to configure digital input for homing: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
    usleep(delay_us_);

    // Not sure if needed:
    // save IO mapping to EEPROM (then power-cycle once)
    if (modbus_write_register(ctx_, 0x1801, 0x2244) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to save IO mapping to EEPROM: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }

    return 0;
}

int ICLStepper::set_as_home() {
    modbus_set_slave(ctx_, slave_id_);
    usleep(delay_us_);

    // Set current position as home (zero) position
    if (modbus_write_register(ctx_, 0x6002, 0x21) == -1) {
        std::cerr << "[Slave " << slave_id_ << "] Failed to set current position as home: "
                  << modbus_strerror(errno) << std::endl;
        return -1;
    }
}
