#ifndef ICL_STEPPER_H
#define ICL_STEPPER_H

#include <modbus/modbus.h>
#include <cstdint>

class ICLStepper {
public:
    ICLStepper(int slave_id, modbus_t* ctx, int pulses_per_revolution = 10000, int gear_ratio = 100);
    int initialize();
    int disable_motor();
    int32_t read_position();
    double get_position_radians();
    int set_position_radians(double position_radians, double radians_per_second);
    int set_position(int target_position, int velocity_rpm, int acceleration, int deceleration);
    int jog(bool clockwise = true);
    int set_jog_velocity(int velocity_rpm);
    int set_slave_id(int slave_id);
    int set_jog_acceleration(int acc);
    uint16_t read_motion_status();
    int home(double home_switch_position = 0.0, double position_after_homing_radians = 0.0, bool clockwise = true, double radians_per_second = 0.5);
    int configure_io_for_homing();
    int set_as_home();
private:
    int slave_id_;
    modbus_t* ctx_;
    int current_position_;
    int delay_us_;
    int pulses_per_revolution_;
    int gear_ratio_;
};

#endif // ICL_STEPPER_H
