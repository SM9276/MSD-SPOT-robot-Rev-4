#include <iostream>
#include <modbus/modbus.h>
#include <math.h>
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
    ICLStepper stepper2(2, ctx, 10000, 100);
    ICLStepper stepper3(3, ctx, 10000, 120);
    ICLStepper stepper4(4, ctx, 10000, 100);
    if (stepper1.initialize() == -1){
        std::cerr << "Failed to initialize stepper 1" << std::endl;
        return -1;
    };

    // home stepper 1
    stepper1.configure_io_for_homing();
    stepper1.home(-M_PI/2, 0, false, 0.15);

    // while homing not complete
    while (((stepper1.read_motion_status() >> 6) & 1) == 0){
        usleep(10);
    }

    std::cout << "Homing motor 1 complete?" << std::endl;

    if (stepper3.initialize() == -1){
        std::cerr << "Failed to initialize stepper 3" << std::endl;
        return -1;
    };

    // home stepper 3
    stepper3.configure_io_for_homing();
    stepper3.home(0.38, 0, false, 0.1);

    // while stepper 1 motion not complete
    while (((stepper1.read_motion_status() >> 2) & 1) == 1){
        usleep(10);
    }
    std::cout << "Moving motor 1 to position PI/2 rad complete?" << std::endl;

    // while homing not complete
    while (((stepper3.read_motion_status() >> 6) & 1) == 0){
        usleep(10);
    }

    std::cout << "Homing motor 3 complete?" << std::endl;

    if (stepper2.initialize() == -1){
        std::cerr << "Failed to initialize stepper 2" << std::endl;
        return -1;
    };

    // home stepper 2
    stepper2.configure_io_for_homing();
    stepper2.home(-1.5, 0, false, 0.1);

    // while homing not complete
    while (((stepper2.read_motion_status() >> 6) & 1) == 0){
        usleep(10);
    }

    std::cout << "Homing motor 2 complete?" << std::endl;

    // // STEPPER 4
    // if (stepper4.initialize() == -1){
    //     std::cerr << "Failed to initialize stepper 4" << std::endl;
    //     return -1;
    // };

    // // home stepper 4
    // stepper4.configure_io_for_homing();
    // stepper4.home(0, 0, true, 0.1);

    // // while homing not complete
    // while (((stepper4.read_motion_status() >> 6) & 1) == 0){
    //     usleep(10);
    // }

    // std::cout << "Homing motor 4 complete?" << std::endl;

    // Move joint 3 to extended position
    stepper3.set_position_radians(M_PI/2, 0.1);

    // while joint 2 motion not complete
    while (((stepper2.read_motion_status() >> 2) & 1) == 1){
        usleep(10);
    }

    // while joint 3 motion not complete
    while (((stepper3.read_motion_status() >> 2) & 1) == 1){
        usleep(10);
    }

    std::cout << "Moving motor 2 to position -PI/2 rad complete?" << std::endl;

    std::cout << "Moving motor 3 to position PI/2 rad complete?" << std::endl;

    usleep(1000000);
    int32_t pos_before = stepper3.read_position();
    std::cout << "Stepper 3 position before setting as home: " << pos_before << std::endl;

    usleep(1000000);

    // // set joint 3 position as home 
    stepper3.set_as_home();

    usleep(1000000);

    int32_t position = stepper3.read_position();
    std::cout << "Stepper 3 position after setting as home (should be 0): " << position << std::endl;

    // clean up
    stepper1.disable_motor();
    stepper2.disable_motor();
    stepper3.disable_motor();
    stepper4.disable_motor();
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}