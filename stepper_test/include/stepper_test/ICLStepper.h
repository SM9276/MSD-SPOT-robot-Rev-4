#ifndef ICL_STEPPER_H
#define ICL_STEPPER_H

#include <modbus/modbus.h>
#include <cstdint>

class ICLStepper {
public:
    ICLStepper(int slave_id, modbus_t* ctx);
    int initialize();
    int set_position(int position, int velocity_rpm = 200, int acc = 50, int dec = 50);
    int get_position() const;
    int jog(bool clockwise = true);
    int set_jog_velocity(int velocity_rpm);
    int set_slave_id(int slave_id);
    int set_jog_acceleration(int acc);
    uint16_t read_motion_status();

private:
    int slave_id_;
    modbus_t* ctx_;
    int current_position_;
    int delay_us_;
};

#endif // ICL_STEPPER_H
