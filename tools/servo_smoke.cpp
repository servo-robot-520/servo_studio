// 无 GUI 冒烟工具：对真实总线做扫描 / 识别 / 状态读取 / 目标下发 /
// 扭矩开关 / 参数读写，验证 FeetechDriver 适配层端到端可用。
// 输出与错误信息使用英文。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "servo_backends/feetech_driver.hpp"

namespace {

    struct Options {
        std::string port;
        int baud_rate = 1000000;
        int timeout_ms = 10;
        int first_id = 0;
        int last_id = 253;
        bool scan_only = false;
        int id = -1;
        double range_deg = 300.0;
        int samples = 5;
        int sample_interval_ms = 200;
        int goal_ticks = -1;
        int goal_time_ms = 0;
        int goal_speed = 1000;
        std::string torque; // on / off
        std::string read_key;
        std::string write_key;
        int write_value = 0;
    };

    void usage(const char* argv0) {
        std::fprintf(stderr,
                     "usage: %s --port DEVICE [options]\n"
                     "  --baud N            bus baud rate (default 1000000)\n"
                     "  --timeout-ms N      response timeout (default 10)\n"
                     "  --scan              scan ids 0..253 and exit\n"
                     "  --id N              target servo id\n"
                     "  --range-deg D       mechanical range for degree conversion (default 300)\n"
                     "  --samples N         state samples to print (default 5)\n"
                     "  --goal T            write goal position in ticks\n"
                     "  --time-ms N         goal run time (default 0)\n"
                     "  --speed N           goal speed in ticks/s (default 1000)\n"
                     "  --torque on|off     torque switch\n"
                     "  --read KEY          read parameter by key\n"
                     "  --write KEY=VALUE   write parameter by key\n",
                     argv0);
    }

    bool next_value(const int argc, char** argv, int& index) {
        if (index + 1 >= argc) {
            std::fprintf(stderr, "error: %s requires a value\n", argv[index]);
            return false;
        }
        ++index;
        return true;
    }

} // namespace

int main(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && next_value(argc, argv, i)) {
            options.port = argv[i];
        }
        else if (arg == "--baud" && next_value(argc, argv, i)) {
            options.baud_rate = std::atoi(argv[i]);
        }
        else if (arg == "--timeout-ms" && next_value(argc, argv, i)) {
            options.timeout_ms = std::atoi(argv[i]);
        }
        else if (arg == "--scan") {
            options.scan_only = true;
        }
        else if (arg == "--id" && next_value(argc, argv, i)) {
            options.id = std::atoi(argv[i]);
        }
        else if (arg == "--range-deg" && next_value(argc, argv, i)) {
            options.range_deg = std::atof(argv[i]);
        }
        else if (arg == "--samples" && next_value(argc, argv, i)) {
            options.samples = std::atoi(argv[i]);
        }
        else if (arg == "--goal" && next_value(argc, argv, i)) {
            options.goal_ticks = std::atoi(argv[i]);
        }
        else if (arg == "--time-ms" && next_value(argc, argv, i)) {
            options.goal_time_ms = std::atoi(argv[i]);
        }
        else if (arg == "--speed" && next_value(argc, argv, i)) {
            options.goal_speed = std::atoi(argv[i]);
        }
        else if (arg == "--torque" && next_value(argc, argv, i)) {
            options.torque = argv[i];
        }
        else if (arg == "--read" && next_value(argc, argv, i)) {
            options.read_key = argv[i];
        }
        else if (arg == "--write" && next_value(argc, argv, i)) {
            const std::string pair = argv[i];
            const auto split = pair.find('=');
            if (split == std::string::npos) {
                std::fprintf(stderr, "error: --write expects KEY=VALUE\n");
                return 2;
            }
            options.write_key = pair.substr(0, split);
            options.write_value = std::atoi(pair.c_str() + split + 1);
        }
        else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        }
        else {
            std::fprintf(stderr, "error: unknown option '%s'\n", arg.c_str());
            usage(argv[0]);
            return 2;
        }
    }
    if (options.port.empty()) {
        usage(argv[0]);
        return 2;
    }

    using namespace servo_backends;
    FeetechDriver driver;
    servo_core::BusConfig config;
    config.port = options.port;
    config.baud_rate = options.baud_rate;
    config.response_timeout_ms = options.timeout_ms;
    if (const auto result = driver.open(config); !result) {
        std::fprintf(stderr, "error: opening '%s' failed: %s\n", options.port.c_str(),
                     result.error().c_str());
        return 1;
    }
    std::printf("opened %s at %d baud\n", options.port.c_str(), options.baud_rate);

    const auto print_info = [](const servo_core::ServoInfo& info) {
        std::printf("servo %3u: model=%s (0x%04X) firmware=%u.%u\n",
                    static_cast<unsigned>(info.id), info.model.c_str(), info.model_number,
                    info.firmware_major, info.firmware_minor);
    };

    // 扫描。
    const auto ids = driver.scan(options.first_id, options.last_id);
    if (!ids) {
        std::fprintf(stderr, "error: scan failed: %s\n", ids.error().c_str());
        return 1;
    }
    std::printf("scan found %zu servo(s)\n", ids.value().size());
    for (const auto& info : ids.value()) {
        print_info(info);
    }
    if (options.scan_only) {
        return 0;
    }
    if (options.id < 0) {
        if (ids.value().size() == 1) {
            options.id = ids.value().front().id;
        }
        else {
            std::fprintf(stderr, "error: --id is required when multiple servos are online\n");
            return 2;
        }
    }

    // 配置换算。
    servo_core::ServoConfig servo_config;
    servo_config.id = static_cast<servo_core::ServoId>(options.id);
    servo_config.range_deg = options.range_deg;
    if (const auto result = driver.set_servo_config(servo_config); !result) {
        std::fprintf(stderr, "error: %s\n", result.error().c_str());
        return 1;
    }
    const auto id = static_cast<servo_core::ServoId>(options.id);

    // 目标下发。
    if (options.goal_ticks >= 0) {
        if (const auto result = driver.write_goal(id, options.goal_ticks, options.goal_time_ms,
                                                  options.goal_speed);
            !result) {
            std::fprintf(stderr, "error: write_goal failed: %s\n", result.error().c_str());
            return 1;
        }
        std::printf("goal written: ticks=%d time=%dms speed=%d tick/s\n", options.goal_ticks,
                    options.goal_time_ms, options.goal_speed);
    }

    // 扭矩开关。
    if (!options.torque.empty()) {
        const bool enable = options.torque == "on";
        if (const auto result = driver.set_torque(id, enable); !result) {
            std::fprintf(stderr, "error: set_torque failed: %s\n", result.error().c_str());
            return 1;
        }
        std::printf("torque %s\n", options.torque.c_str());
    }

    // 参数写入 / 读取。
    if (!options.write_key.empty()) {
        if (const auto result = driver.write_param(id, options.write_key, options.write_value);
            !result) {
            std::fprintf(stderr, "error: write_param failed: %s\n", result.error().c_str());
            return 1;
        }
        std::printf("param written: %s=%d\n", options.write_key.c_str(), options.write_value);
    }
    if (!options.read_key.empty()) {
        const auto value = driver.read_param(id, options.read_key);
        if (!value) {
            std::fprintf(stderr, "error: read_param failed: %s\n", value.error().c_str());
            return 1;
        }
        std::printf("param read: %s=%d\n", options.read_key.c_str(), value.value());
    }

    // 状态采样。
    for (int sample = 0; sample < options.samples; ++sample) {
        const auto state = driver.read_state(id);
        if (!state) {
            std::fprintf(stderr, "error: read_state failed: %s\n", state.error().c_str());
            return 1;
        }
        std::printf(
            "state[%d]: pos=%.1f tick (%.2f deg) vel=%.0f tick/s (%.1f deg/s) load=%.3f "
            "V=%.2f T=%.0fC I=%.3fA moving=%d\n",
            sample, state.value().position_ticks, state.value().position_deg,
            state.value().velocity_ticks_s, state.value().velocity_deg_s, state.value().load,
            state.value().voltage, state.value().temperature, state.value().current,
            state.value().moving ? 1 : 0);
        if (sample + 1 < options.samples) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.sample_interval_ms));
        }
    }
    return 0;
}
