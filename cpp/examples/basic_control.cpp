/*
 * RobStride C++ Basic Control Example
 * Demonstrates basic motor control operations
 * Self-contained: includes all necessary CAN and protocol functions
 */

#include "../include/protocol.h"
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <atomic>

// Linux SocketCAN headers
#include <stdio.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- Global flag for clean exit ---
std::atomic<bool> running(true);

void signal_handler(int signum) {
    std::cout << "\n🛑 Caught signal, shutting down..." << std::endl;
    running = false;
}

// --- CAN Helper Functions ---

int init_can(const char* ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return -1;
    }

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame;
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    if (data) {
        memcpy(frame.data, data, dlc);
    } else {
        memset(frame.data, 0, 8);
    }

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("write");
        return false;
    }
    return true;
}

bool read_frame(int s, struct can_frame* frame, int timeout_ms = 100) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);

    int ret = select(s + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1) {
        perror("select");
        return false;
    } else if (ret == 0) {
        return false;
    }

    if (read(s, frame, sizeof(struct can_frame)) < 0) {
        perror("read");
        return false;
    }
    return true;
}

// --- RobStride Protocol Functions ---

const uint32_t HOST_ID = 0xFF;

bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (CommType::ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool disable_motor(int s, int motor_id) {
    uint32_t ext_id = (CommType::DISABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool set_mode_raw(int s, int motor_id, int8_t mode) {
    uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    
    // Pack param ID (little endian)
    data[0] = ParamID::MODE & 0xFF;
    data[1] = (ParamID::MODE >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    // Pack value (int8)
    data[4] = static_cast<uint8_t>(mode);
    
    return send_frame(s, ext_id, data, 8);
}

bool write_limit(int s, int motor_id, uint16_t param_id, float limit) {
    uint32_t ext_id = (CommType::WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    
    // Pack param ID (little endian)
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    // Pack float value (little endian)
    memcpy(&data[4], &limit, sizeof(float));
    
    return send_frame(s, ext_id, data, 8);
}

bool write_operation_frame(int s, int motor_id, double pos, double kp_val, double kd_val) {
    // Scaling values for RS-03
    const double POS_SCALE = 4 * M_PI;
    const double VEL_SCALE = 50.0;
    const double KP_SCALE = 5000.0;
    const double KD_SCALE = 100.0;
    const double TQ_SCALE = 60.0;

    double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
    double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
    double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));
    
    uint16_t pos_u16 = static_cast<uint16_t>(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
    uint16_t vel_u16 = 0x7FFF; // 0 velocity
    uint16_t kp_u16 = static_cast<uint16_t>((kp_clamped / KP_SCALE) * 0xFFFF);
    uint16_t kd_u16 = static_cast<uint16_t>((kd_clamped / KD_SCALE) * 0xFFFF);
    uint16_t torque_u16 = 0x7FFF; // 0 torque_ff

    uint8_t data[8];
    // Pack as big endian
    data[0] = (pos_u16 >> 8) & 0xFF;
    data[1] = pos_u16 & 0xFF;
    data[2] = (vel_u16 >> 8) & 0xFF;
    data[3] = vel_u16 & 0xFF;
    data[4] = (kp_u16 >> 8) & 0xFF;
    data[5] = kp_u16 & 0xFF;
    data[6] = (kd_u16 >> 8) & 0xFF;
    data[7] = kd_u16 & 0xFF;
    
    uint32_t ext_id = (CommType::OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    
    return send_frame(s, ext_id, data, 8);
}

bool wait_for_status(int s, int timeout_ms = 100) {
    struct can_frame frame;
    auto start = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        if (read_frame(s, &frame, 10)) {
            if (frame.can_id & CAN_EFF_FLAG) {
                uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
                if (comm_type == CommType::OPERATION_STATUS) {
                    return true;
                }
            }
        }
    }
    return false;
}

int main(int argc, char* argv[]) {
    int motor_id = 11;
    std::string can_interface = "can0";
    
    if (argc > 1) {
        motor_id = std::atoi(argv[1]);
    }
    if (argc > 2) {
        can_interface = argv[2];
    }
    const char* env_can = std::getenv("CAN_INTERFACE");
    if (env_can) {
        can_interface = env_can;
    }

    std::cout << "🎯 RobStride C++ Basic Control Example" << std::endl;
    std::cout << "Using Motor ID: " << motor_id << std::endl;
    std::cout << "Using CAN interface: " << can_interface << std::endl;

    // Check if using SLCAN device (serial)
    if (can_interface.rfind("/dev/tty", 0) == 0 || can_interface.rfind("slcan", 0) == 0) {
        std::cerr << "❌ SLCAN serial device detected: " << can_interface << std::endl;
        std::cerr << "Please run slcand first to create a CAN interface:" << std::endl;
        std::cerr << "  sudo slcand -o -s8 -S 1000000 " << can_interface << " can0" << std::endl;
        std::cerr << "  sudo ip link set up can0" << std::endl;
        std::cerr << "Then use 'can0' as the interface." << std::endl;
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int s = init_can(can_interface.c_str());
    if (s < 0) {
        std::cerr << "❌ Failed to initialize CAN interface: " << can_interface << std::endl;
        std::cerr << "Make sure the interface exists and is up:" << std::endl;
        std::cerr << "  sudo ip link set " << can_interface << " type can bitrate 1000000" << std::endl;
        std::cerr << "  sudo ip link set up " << can_interface << std::endl;
        return 1;
    }

    std::cout << "✅ CAN interface initialized" << std::endl;

    try {
        // Enable motor
        std::cout << "⚡ Enabling motor..." << std::endl;
        enable_motor(s, motor_id);
        wait_for_status(s, 500);
        usleep(500000);

        // Set MIT mode
        std::cout << "⚙️ Setting MIT mode..." << std::endl;
        set_mode_raw(s, motor_id, ControlMode::MIT_MODE);
        usleep(200000);

        // Set limits
        std::cout << "📊 Setting limits..." << std::endl;
        write_limit(s, motor_id, ParamID::VELOCITY_LIMIT, 20.0);
        wait_for_status(s);
        write_limit(s, motor_id, ParamID::TORQUE_LIMIT, 20.0);
        wait_for_status(s);

        // Initial position
        std::cout << "🏠 Setting home position..." << std::endl;
        write_operation_frame(s, motor_id, 0.0, 30.0, 0.5);
        wait_for_status(s);

        // Test movements
        std::cout << "🎮 Starting test movements..." << std::endl;

        double movements[] = {0.0, M_PI/2, -M_PI/2, M_PI, 0.0};
        const char* descriptions[] = {"Home (0°)", "90°", "-90°", "180°", "Home (0°)"};

        for (int i = 0; i < 5; i++) {
            if (!running) break;
            std::cout << "📍 Moving to " << descriptions[i] << std::endl;
            write_operation_frame(s, motor_id, movements[i], 30.0, 0.5);
            wait_for_status(s);
            sleep(2);
        }

        // Return to zero and disable
        std::cout << "🏠 Returning to zero..." << std::endl;
        write_operation_frame(s, motor_id, 0.0, 30.0, 0.5);
        wait_for_status(s, 1000);
        sleep(1);
        
        std::cout << "🚫 Disabling motor..." << std::endl;
        disable_motor(s, motor_id);
        wait_for_status(s);

        std::cout << "✅ Test completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }

    close(s);
    return 0;
}