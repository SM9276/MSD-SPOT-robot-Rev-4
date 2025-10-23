#include <iostream>
#include <modbus/modbus.h>
#include "marge/ICLStepper.h"


int main(){
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
    if (ctx == nullptr) {
        std::cerr << "Unable to create the libmodbus context" << std::endl;
        return -1;
    }
    modbus_set_response_timeout(ctx, 1, 0);
    usleep(10'000);  // Wait 10 ms
    if (modbus_connect(ctx) == -1) {
        std::cerr << "Connection failed: " << modbus_strerror(errno) << std::endl;
        modbus_free(ctx);
        return -1;
    }
    usleep(10'000);  // Wait 10 ms
    ICLStepper stepper1(1, ctx, 10000, 51);
    // ICLStepper stepper2(2, ctx, 10000, 100);
    // ICLStepper stepper3(3, ctx, 10000, 120);
    // ICLStepper stepper4(4, ctx, 10000, 100);
    if (stepper1.initialize() == -1){
        std::cerr << "Failed to initialize stepper 1" << std::endl;
        return -1;
    };

    // home stepper 1
    stepper1.configure_io_for_homing();
    stepper1.home(0, false, 0.1);

    // while homing not complete
    while (((stepper1.read_motion_status() >> 6) & 1) == 0){
        usleep(10);
    }

    std::cout << "Homing complete?" << std::endl;

    // clean up
    stepper1.disable_motor();
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}