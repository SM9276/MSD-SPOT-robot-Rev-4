#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <cctype>
#include "stepper_test/ICLStepper.h"
#include <map>
#include "rclcpp/rclcpp.hpp"

struct RawTerm {
    termios orig{};
    bool active{false};

    void enable() {
        if (!isatty(STDIN_FILENO)) return;
        tcgetattr(STDIN_FILENO, &orig);
        termios raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO); // raw mode: no canonical, no echo
        raw.c_cc[VMIN] = 0;              // non-blocking
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active = true;
    }
    void disable() {
        if (active) tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        active = false;
    }
    ~RawTerm() { disable(); }
};

int main() {
    std::cout << R"(Stepper control ready.
Digits [0-9]: select motor slave ID.
Arrow keys: toggle jogging (left/right) and adjust speed ±100 (up/down).
Keys [a-;]: apply preset jog velocities (low to high).
Keys [z-m]: move to preset absolute positions.
w: read motion status; e: home; r: read position; t: set current position as home.
Press 'q' or Esc to quit.
)";
    RawTerm rt;
    rt.enable();

    std::map<char, int> key_velocity_map = {
        {'a', 200}, {'s', 224}, {'d', 253}, {'f', 284}, {'g', 299},
        {'h', 336}, {'j', 376}, {'k', 400}, {'l', 449}, {';', 506}
    };
    for (auto& kv : key_velocity_map) {
        kv.second *= 2;
    }
    int motor_slave_id = 1;
    int target_velocity = 100;
    bool jogging = false;
    bool clockwise = true;
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
    ICLStepper stepper(motor_slave_id, ctx, 10000, 100);
    stepper.initialize();
    stepper.set_jog_velocity(target_velocity);
    stepper.set_jog_acceleration(4000);

    while (true) {
        fd_set set;
        FD_ZERO(&set);
        FD_SET(STDIN_FILENO, &set);
        timeval tv{0, 50'000}; // 50 ms
        int ready = select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv);
        if (ready > 0 && FD_ISSET(STDIN_FILENO, &set)) {
            unsigned char buf[8];
            ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
            for (ssize_t i = 0; i < n; ++i) {
                int ch = buf[i];

                if (std::isdigit(ch)) {
                    std::cout << "Digit pressed: " << static_cast<char>(ch) << "\n";
                    motor_slave_id = ch - '0';
                    std::cout << "Selected motor slave ID: " << motor_slave_id << "\n";
                    stepper.set_slave_id(motor_slave_id);
                    stepper.set_jog_velocity(target_velocity);
                    stepper.set_jog_acceleration(4000);
                }

                else if (ch == '\033') { // Escape sequence
                    if (n > i + 2 && buf[i + 1] == '[') {
                        if (buf[i + 2] == 'C') { // Right arrow
                            std::cout << "Right arrow pressed.\n";
                            jogging = not jogging;
                            clockwise = true;
                            i += 2; // Skip the rest of the escape sequence
                        } else if (buf[i + 2] == 'D') { // Left arrow
                            std::cout << "Left arrow pressed.\n";
                            jogging = not jogging;
                            clockwise = false;
                            i += 2; // Skip the rest of the escape sequence
                        } else if (buf[i + 2] == 'A') { // Up arrow
                            std::cout << "Up arrow pressed.\n";
                            target_velocity += 100;
                            stepper.set_jog_velocity(target_velocity);
                            std::cout << "Increased target velocity to " << target_velocity << "\n";
                            i += 2; // Skip the rest of the escape sequence
                        } else if (buf[i + 2] == 'B') { // Down arrow
                            std::cout << "Down arrow pressed.\n";
                            target_velocity = std::max(0, target_velocity - 100);
                            stepper.set_jog_velocity(target_velocity);
                            std::cout << "Decreased target velocity to " << target_velocity << "\n";
                            i += 2; // Skip the rest of the escape sequence
                        }
                    }
                }

                else if (key_velocity_map.find(ch) != key_velocity_map.end()) {
                    target_velocity = key_velocity_map[ch];
                    stepper.set_jog_velocity(target_velocity);
                    std::cout << "Set target velocity to " << target_velocity << " for key '" << static_cast<char>(ch) << "'\n";
                }

                else if (ch == 'z' || ch == 'x' || ch == 'c' || ch == 'v' || ch == 'b' || ch == 'n' || ch == 'm') {
                    std::cout << "Key '" << static_cast<char>(ch) << "' pressed, move to position.\n";
                    if (ch == 'v') {
                        stepper.set_position_radians(0, 3.14);
                    } else if (ch == 'b') {
                        stepper.set_position_radians(3.14, 3.14);
                    } else if (ch == 'n') {
                        stepper.set_position_radians(3.14*2, 3.14);
                    } else if (ch == 'm') {
                        stepper.set_position_radians(3.14*3, 3.14);
                    } else if (ch == 'c') {
                        stepper.set_position_radians(-3.14, 3.14);
                    } else if (ch == 'x') {
                        stepper.set_position_radians(-3.14*2, 3.14);
                    } else if (ch == 'z') {
                        stepper.set_position_radians(-3.14*3, 3.14);
                    }
                }

                else if (ch == 'w') {
                    std::cout << "Read motion status.\n";
                    stepper.read_motion_status();
                }

                else if (ch == 'e') {
                    std::cout << "Triggering homing.\n";
                    stepper.configure_io_for_homing();
                    stepper.home();
                }

                else if (ch == 'r') {
                    std::cout << "Reading position.\n";
                    int32_t pos = stepper.read_position();
                    std::cout << "Current position (pulses): " << pos << "\n";
                    double pos_rad = stepper.get_position_radians();
                    std::cout << "Current position (radians): " << pos_rad << "\n";
                }

                else if (ch == 't') {
                    std::cout << "Setting current position as home (zero) position.\n";
                    stepper.set_as_home();
                }

                else if (ch == 'q') {
                    std::cout << "Quit.\n";
                    return 0;
                }
            }
        }
        if (jogging){
            stepper.jog(clockwise);
        }
    }
}
