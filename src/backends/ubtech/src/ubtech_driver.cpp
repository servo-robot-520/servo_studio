#include "servo_backends/ubtech_driver.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>

#include "servo_core/registry.hpp"

namespace servo_backends {
    namespace {
        using servo_core::BusConfig;

        constexpr uint8_t kHeader1 = 0xFA;
        constexpr uint8_t kHeader2 = 0xAF;
        constexpr uint8_t kFwHeader1 = 0xFC;
        constexpr uint8_t kFwHeader2 = 0xCF;
        constexpr uint8_t kTail = 0xED;
        constexpr uint8_t kAck = 0xAA;
        constexpr uint8_t kNack = 0xEE;

        constexpr size_t kFrameSize = 10;

        // 命令码（帧字节 3）。
        constexpr uint8_t kCmdMove = 0x01;        // 0x01 + FF = 强制中止
        constexpr uint8_t kStopMarker = 0xFF;
        constexpr uint8_t kCmdReadAngle = 0x02;
        constexpr uint8_t kCmdSetId = 0xCD;
        constexpr uint8_t kCmdSetOffset = 0xD2;
        constexpr uint8_t kCmdReadOffset = 0xD4;
        constexpr uint8_t kCmdReadVersion = 0x01; // FC CF 帧头下

        constexpr int kMaxAngleDeg = 240;
        constexpr int kMaxMoveUnits = 255;   // 255 × 20ms
        constexpr int kMaxLockUnits = 3270;  // 3270 × 20ms
        constexpr int kTimeQuantumMs = 20;

        constexpr int kMinServoId = 1;
        constexpr int kMaxServoId = 240; // 0 为广播，不参与扫描/点名

        speed_t to_termios_baud(const int baud_rate) {
            // 标准速率 + 常见扩展速率（扩展项按平台宏裁剪；
            // UBTECH 协议实测为 115200，其余速率仅按通用串口能力提供）。
            switch (baud_rate) {
                case 9600: return B9600;
                case 19200: return B19200;
                case 38400: return B38400;
                case 57600: return B57600;
                case 115200: return B115200;
                case 230400: return B230400;
#ifdef B76800
                case 76800: return B76800;
#endif
#ifdef B500000
                case 500000: return B500000;
#endif
#ifdef B1000000
                case 1000000: return B1000000;
#endif
                default: return static_cast<speed_t>(0);
            }
        }

        uint8_t checksum(const uint8_t* frame) {
            uint8_t sum = 0;
            for (int i = 2; i <= 7; ++i) {
                sum = static_cast<uint8_t>(sum + frame[i]);
            }
            return sum;
        }
    } // namespace

    UbtechDriver::~UbtechDriver() {
        UbtechDriver::close();
    }

    void UbtechDriver::set_trace_hook(TraceHook hook) {
        trace_hook_ = std::move(hook);
    }

    Result UbtechDriver::open(const BusConfig& config) {
        if (fd_ >= 0) {
            return Result::failure("Serial port is already open");
        }
        const speed_t baud = to_termios_baud(config.baud_rate);
        if (baud == 0) {
            return Result::failure("Unsupported baud rate " + std::to_string(config.baud_rate));
        }
        const int fd = ::open(config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            return Result::failure("Failed to open '" + config.port + "': " +
                                   std::strerror(errno));
        }
        termios tty{};
        if (::tcgetattr(fd, &tty) != 0) {
            const std::string error = std::strerror(errno);
            ::close(fd);
            return Result::failure("tcgetattr failed: " + error);
        }
        cfmakeraw(&tty);
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
            const std::string error = std::strerror(errno);
            ::close(fd);
            return Result::failure("tcsetattr failed: " + error);
        }
        ::tcflush(fd, TCIOFLUSH);
        fd_ = fd;
        timeout_ms_ = std::max(1, config.response_timeout_ms);
        return Result::success();
    }

    void UbtechDriver::close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool UbtechDriver::is_open() const {
        return fd_ >= 0;
    }

    Result UbtechDriver::set_servo_config(const ServoConfig& config) {
        // UBTECH 的换算固定（tick 即度、协议钳位 0..240），仅记录不参与换算。
        configs_[config.id] = config;
        return Result::success();
    }

    TelemetryChannels UbtechDriver::telemetry_channels(const ServoId) const {
        // 无连续遥测（角度回读有失电副作用，仅手动单次读取）。
        return TelemetryChannels::none();
    }

    DriverCapabilities UbtechDriver::capabilities() const {
        return {.torque_damping = false,
                .polling_safe = false,
                .uses_protocol_panel = true};
    }

    const ServoConfig& UbtechDriver::config_for(const ServoId id) const {
        static const ServoConfig kDefault{};
        if (const auto it = configs_.find(id); it != configs_.end()) {
            return it->second;
        }
        return kDefault;
    }

    void UbtechDriver::trace(const char* direction, const uint8_t* data, const size_t size) const {
        if (trace_hook_) {
            trace_hook_(direction, std::vector<uint8_t>(data, data + size));
        }
    }

    UbtechDriver::Frame UbtechDriver::build_frame(const uint8_t header1, const uint8_t header2,
                                                  const ServoId id, const uint8_t command,
                                                  const uint8_t p1h, const uint8_t p1l,
                                                  const uint8_t p2h, const uint8_t p2l) {
        const Frame frame = {header1, header2, id, command, p1h, p1l, p2h, p2l, 0, kTail};
        auto frame_copy = frame;
        frame_copy[8] = checksum(frame_copy.data());
        return frame_copy;
    }

    Result UbtechDriver::send_frame(const Frame& frame) const {
        if (fd_ < 0) {
            return Result::failure("Serial port is not open");
        }
        trace("tx", frame.data(), frame.size());
        size_t written = 0;
        while (written < frame.size()) {
            const ssize_t n = ::write(fd_, frame.data() + written, frame.size() - written);
            if (n < 0) {
                if (errno == EAGAIN || errno == EINTR) {
                    pollfd pfd{.fd = fd_, .events = POLLOUT, .revents = 0};
                    if (::poll(&pfd, 1, timeout_ms_) <= 0) {
                        return Result::failure("Serial write timed out");
                    }
                    continue;
                }
                return Result::failure(std::string("Serial write failed: ") +
                                       std::strerror(errno));
            }
            written += static_cast<size_t>(n);
        }
        return Result::success();
    }

    Expected<std::vector<uint8_t>> UbtechDriver::read_registers(const ServoId, const uint8_t,
                                                                const size_t) {
        return Expected<std::vector<uint8_t>>::failure(
            "Raw register access is not supported: UBTECH protocol has no register map");
    }

    Result UbtechDriver::write_registers(const ServoId, const uint8_t,
                                         const std::vector<uint8_t>&) {
        return Result::failure(
            "Raw register access is not supported: UBTECH protocol has no register map");
    }

    Expected<UbtechDriver::Frame> UbtechDriver::receive_frame(const uint8_t header1,
                                                             const uint8_t header2) const {
        // 缓冲区解析：跳过噪声字节，找到帧头后凑满 10 字节，校验帧尾与校验和。
        std::vector<uint8_t> buffer;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
        while (std::chrono::steady_clock::now() < deadline) {
            if (buffer.size() >= 2) {
                // 找到帧头并凑齐整帧。
                for (size_t start = 0; start + 1 < buffer.size(); ++start) {
                    if (buffer[start] != header1 || buffer[start + 1] != header2) {
                        continue;
                    }
                    if (buffer.size() - start < kFrameSize) {
                        break; // 帧未收齐，继续读。
                    }
                    Frame frame{};
                    std::copy(buffer.begin() + static_cast<long>(start),
                              buffer.begin() + static_cast<long>(start + kFrameSize),
                              frame.begin());
                    trace("rx", frame.data(), frame.size());
                    if (frame[9] != kTail) {
                        return Expected<Frame>::failure("Response frame tail mismatch");
                    }
                    if (frame[8] != checksum(frame.data())) {
                        return Expected<Frame>::failure("Response checksum mismatch");
                    }
                    return Expected<Frame>(frame);
                }
            }
            // 读取更多字节（按剩余时间等待）。
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
            if (const int wait_ms = static_cast<int>(std::max<int64_t>(1, remaining.count())); ::poll(&pfd, 1, wait_ms) <= 0) {
                continue;
            }
            uint8_t chunk[64];
            if (const ssize_t n = ::read(fd_, chunk, sizeof(chunk)); n > 0) {
                buffer.insert(buffer.end(), chunk, chunk + n);
            }
        }
        return Expected<Frame>::failure("Timed out waiting for response frame");
    }

    Result UbtechDriver::receive_short_ack() const {
        // 短回应：0xAA [+舵机ID]。协议规定失败/错误不回应，超时即失败。
        std::vector<uint8_t> buffer;
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
        while (buffer.empty() && std::chrono::steady_clock::now() < deadline) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
            if (const int wait_ms = static_cast<int>(std::max<int64_t>(1, remaining.count())); ::poll(&pfd, 1, wait_ms) <= 0) {
                continue;
            }
            uint8_t chunk[8];
            if (const ssize_t n = ::read(fd_, chunk, sizeof(chunk)); n > 0) {
                buffer.insert(buffer.end(), chunk, chunk + n);
            }
        }
        if (buffer.empty()) {
            return Result::failure(
                "No response (servo offline, or ignored the command during lock time)");
        }
        trace("rx", buffer.data(), buffer.size());
        if (buffer[0] != kAck) {
            return Result::failure("Unexpected ack byte 0x" +
                                   std::to_string(buffer[0]));
        }
        // 兼容 0xAA 与 0xAA+ID 两种形态：把随后即到的 ID 字节收掉，
        // 避免污染下一条回应（0xAA+ID 两字节间隔远小于超时）。
        if (buffer.size() == 1) {
            pollfd pfd{.fd = fd_, .events = POLLIN, .revents = 0};
            if (::poll(&pfd, 1, 2) > 0) {
                uint8_t extra[8];
                if (const ssize_t n = ::read(fd_, extra, sizeof(extra)); n > 0) {
                    trace("rx", extra, static_cast<size_t>(n));
                }
            }
        }
        return Result::success();
    }

    Result UbtechDriver::cmd_move(const ServoId id, const uint8_t angle_deg,
                                  const uint8_t move_units, const uint16_t lock_units) const {
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdMove, angle_deg, move_units,
                                        static_cast<uint8_t>(lock_units >> 8),
                                        static_cast<uint8_t>(lock_units & 0xFF));
        if (const auto result = send_frame(frame); !result) {
            return result;
        }
        return receive_short_ack();
    }

    Result UbtechDriver::cmd_stop(const ServoId id) const {
        // 强制中止：0x01 + FF 00 00 00；立即停转并失电（仅靠齿轮阻尼维持位置）。
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdMove, kStopMarker, 0, 0, 0);
        if (const auto result = send_frame(frame); !result) {
            return result;
        }
        return receive_short_ack();
    }

    Expected<std::pair<int, int>> UbtechDriver::cmd_read_angle(const ServoId id) const {
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdReadAngle, 0, 0, 0, 0);
        if (const auto result = send_frame(frame); !result) {
            return Expected<std::pair<int, int>>::failure(result.error());
        }
        const auto response = receive_frame(kHeader1, kHeader2);
        if (!response) {
            return Expected<std::pair<int, int>>::failure(response.error());
        }
        const Frame& r = response.value();
        const int target = (r[4] << 8) | r[5];
        const int actual = (r[6] << 8) | r[7];
        return Expected<std::pair<int, int>>(std::make_pair(target, actual));
    }

    Result UbtechDriver::cmd_set_id(const ServoId id, const ServoId new_id) const {
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdSetId, 0, new_id, 0, 0);
        if (const auto result = send_frame(frame); !result) {
            return result;
        }
        const auto response = receive_frame(kHeader1, kHeader2);
        if (!response) {
            return Result::failure(response.error());
        }
        if (response.value()[3] == kNack) {
            return Result::failure("Servo rejected ID change");
        }
        return Result::success();
    }

    Result UbtechDriver::cmd_set_offset(const ServoId id, const int offset) const {
        const auto raw = static_cast<uint16_t>(static_cast<int16_t>(offset));
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdSetOffset, 0, 0,
                                        static_cast<uint8_t>(raw >> 8),
                                        static_cast<uint8_t>(raw & 0xFF));
        if (const auto result = send_frame(frame); !result) {
            return result;
        }
        const auto response = receive_frame(kHeader1, kHeader2);
        if (!response) {
            return Result::failure(response.error());
        }
        if (response.value()[3] == kNack) {
            return Result::failure("Servo rejected angle offset");
        }
        return Result::success();
    }

    Expected<int> UbtechDriver::cmd_read_offset(const ServoId id) const {
        const Frame frame = build_frame(kHeader1, kHeader2, id, kCmdReadOffset, 0, 0, 0, 0);
        if (const auto result = send_frame(frame); !result) {
            return Expected<int>::failure(result.error());
        }
        const auto response = receive_frame(kHeader1, kHeader2);
        if (!response) {
            return Expected<int>::failure(response.error());
        }
        const Frame& r = response.value();
        const auto raw = static_cast<uint16_t>((r[6] << 8) | r[7]);
        return Expected<int>(static_cast<int>(static_cast<int16_t>(raw)));
    }

    Expected<std::array<uint8_t, 4>> UbtechDriver::cmd_read_version(const ServoId id) const {
        const Frame frame = build_frame(kFwHeader1, kFwHeader2, id, kCmdReadVersion, 0, 0, 0, 0);
        if (const auto result = send_frame(frame); !result) {
            return Expected<std::array<uint8_t, 4>>::failure(result.error());
        }
        const auto response = receive_frame(kFwHeader1, kFwHeader2);
        if (!response) {
            return Expected<std::array<uint8_t, 4>>::failure(response.error());
        }
        const Frame& r = response.value();
        return Expected<std::array<uint8_t, 4>>(std::array<uint8_t, 4>{r[4], r[5], r[6], r[7]});
    }

    Expected<std::vector<ServoInfo>> UbtechDriver::scan(const int first_id, const int last_id) {
        if (fd_ < 0) {
            return Expected<std::vector<ServoInfo>>::failure("Serial port is not open");
        }
        // 探测命令用固件版本读取（FC CF 0x01）：无副作用、一问一答即完成身份识别。
        // 0 为广播、>240 非法，统一跳过。
        std::vector<ServoInfo> infos;
        for (int id = std::max(first_id, kMinServoId); id <= std::min(last_id, kMaxServoId);
             ++id) {
            const auto version = cmd_read_version(static_cast<ServoId>(id));
            if (!version) {
                continue; // 无回应 = 不在线，属正常扫描噪声。
            }
            ServoInfo info;
            info.id = static_cast<ServoId>(id);
            info.model = "UBTECH";
            info.model_number = 0; // 协议无型号读取。
            info.firmware_major = version.value()[0];
            info.firmware_minor = version.value()[1];
            info.max_range_deg = kMaxAngleDeg;
            info.channels = TelemetryChannels::none();
            infos.push_back(info);
        }
        return Expected<std::vector<ServoInfo>>(infos);
    }

    Expected<ServoInfo> UbtechDriver::identify(const ServoId id) {
        const auto version = cmd_read_version(id);
        if (!version) {
            return Expected<ServoInfo>::failure(version.error());
        }
        ServoInfo info;
        info.id = id;
        info.model = "UBTECH";
        info.model_number = 0;
        info.firmware_major = version.value()[0];
        info.firmware_minor = version.value()[1];
        info.max_range_deg = kMaxAngleDeg;
        info.channels = TelemetryChannels::none();
        return Expected<ServoInfo>(info);
    }

    Expected<ServoState> UbtechDriver::read_state(const ServoId id) {
        // 副作用警告：0x02 回应后舵机失电，仅靠齿轮阻尼维持位置。
        const auto angles = cmd_read_angle(id);
        if (!angles) {
            return Expected<ServoState>::failure(angles.error());
        }
        ServoState state;
        state.id = id;
        // tick 即度（协议绝对角 0..240）；仅单次回读有意义，其余通道无遥测。
        state.position_ticks = angles.value().second;
        state.position_deg = angles.value().second;
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        state.velocity_ticks_s = nan;
        state.velocity_deg_s = nan;
        state.load = nan;
        state.voltage = nan;
        state.temperature = nan;
        state.current = nan;
        state.moving = angles.value().first != angles.value().second;
        return Expected<ServoState>(state);
    }

    Result UbtechDriver::write_goal(const ServoId id, const int position_ticks, const int time_ms,
                                    const int speed_ticks_s, const int acc, const int lock_ms) {
        (void)speed_ticks_s; // 协议无速度概念，运动快慢由运动时间表达。
        (void)acc;
        const auto& config = config_for(id);
        const int lower = std::clamp(static_cast<int>(std::lround(config.min_ticks)), 0,
                                     kMaxAngleDeg);
        const int upper = std::clamp(static_cast<int>(std::lround(config.max_ticks)), 0,
                                     kMaxAngleDeg);
        const int angle = std::clamp(position_ticks, lower, upper);
        const int move_units =
            std::clamp(static_cast<int>(std::lround(time_ms / static_cast<double>(kTimeQuantumMs))), 0,
                       kMaxMoveUnits);
        const int lock_units =
            std::clamp(static_cast<int>(std::lround(lock_ms / static_cast<double>(kTimeQuantumMs))), 0,
                       kMaxLockUnits);
        return cmd_move(id, static_cast<uint8_t>(angle), static_cast<uint8_t>(move_units),
                        static_cast<uint16_t>(lock_units));
    }

    Result UbtechDriver::sync_write_goals(const std::vector<ServoId>& ids,
                                          const std::vector<int>& positions_ticks,
                                          const std::vector<int>& times_ms,
                                          const std::vector<int>& speeds_ticks_s,
                                          const int acc) {
        if (ids.size() != positions_ticks.size() || ids.size() != times_ms.size() ||
            ids.size() != speeds_ticks_s.size()) {
            return Result::failure(
                "sync_write_goals requires matching ids, positions, times and speeds");
        }
        // 协议无 SYNC WRITE：逐条下发（各自回锁），时序上近似同步。
        for (size_t i = 0; i < ids.size(); ++i) {
            if (const auto result = write_goal(ids[i], positions_ticks[i], times_ms[i],
                                              speeds_ticks_s[i], acc, 0);
                !result) {
                return result;
            }
        }
        return Result::success();
    }

    Result UbtechDriver::set_torque(const ServoId id, const bool enabled) {
        if (enabled) {
            // 协议无法凭命令上电锁紧：下发目标即上电并保持。
            return Result::failure(
                "UBTECH cannot enable torque on command; send a goal to energize the servo");
        }
        // 松扭矩 = 强制中止（0x01 FF）：立即停转并失电。
        return cmd_stop(id);
    }

    std::vector<ParamDescriptor> UbtechDriver::parameters() const {
        // 协议参数极少；固件版本 4 字节语义未知，按原样拆为 4 个只读参数展示。
        return {
            ParamDescriptor{.key = "id",
                            .label = "ID",
                            .zh = "舵机 ID",
                            .unit = "",
                            .access = servo_core::ParamAccess::kRam,
                            .area = servo_core::ParamArea::kDefault,
                            .address = kCmdSetId,
                            .min = kMinServoId,
                            .max = kMaxServoId,
                            .choices = {}},
            ParamDescriptor{.key = "angle_offset",
                            .label = "Angle offset",
                            .zh = "角度偏移",
                            .unit = "1/3°",
                            .access = servo_core::ParamAccess::kRam,
                            .area = servo_core::ParamArea::kSram,
                            .address = kCmdSetOffset,
                            .min = -90,
                            .max = 90,
                            .choices = {}},
            ParamDescriptor{.key = "firmware_v1",
                            .label = "Firmware version 1",
                            .zh = "版本号 1",
                            .unit = "",
                            .access = servo_core::ParamAccess::kReadOnly,
                            .area = servo_core::ParamArea::kDefault,
                            .address = kCmdReadVersion,
                            .min = 0,
                            .max = 255,
                            .choices = {}},
            ParamDescriptor{.key = "firmware_v2",
                            .label = "Firmware version 2",
                            .zh = "版本号 2",
                            .unit = "",
                            .access = servo_core::ParamAccess::kReadOnly,
                            .area = servo_core::ParamArea::kDefault,
                            .address = kCmdReadVersion,
                            .min = 0,
                            .max = 255,
                            .choices = {}},
            ParamDescriptor{.key = "firmware_v3",
                            .label = "Firmware version 3",
                            .zh = "版本号 3",
                            .unit = "",
                            .access = servo_core::ParamAccess::kReadOnly,
                            .area = servo_core::ParamArea::kDefault,
                            .address = kCmdReadVersion,
                            .min = 0,
                            .max = 255,
                            .choices = {}},
            ParamDescriptor{.key = "firmware_v4",
                            .label = "Firmware version 4",
                            .zh = "版本号 4",
                            .unit = "",
                            .access = servo_core::ParamAccess::kReadOnly,
                            .area = servo_core::ParamArea::kDefault,
                            .address = kCmdReadVersion,
                            .min = 0,
                            .max = 255,
                            .choices = {}},
        };
    }

    Expected<int> UbtechDriver::read_param(const ServoId id, const std::string& key) {
        if (fd_ < 0) {
            return Expected<int>::failure("Serial port is not open");
        }
        if (key == "id") {
            // 协议无 ID 查询命令；回读即当前点名使用的 ID。
            return Expected<int>(id);
        }
        if (key == "angle_offset") {
            return cmd_read_offset(id);
        }
        if (key.rfind("firmware_v", 0) == 0) {
            // 版本号下标解析（strtol 报告转换失败；先校验再发总线请求）。
            char* end = nullptr;
            const long parsed = std::strtol(key.c_str() + 10, &end, 10);
            if (end == key.c_str() + 10 || parsed < 1 || parsed > 4) {
                return Expected<int>::failure("Unknown parameter '" + key + "'");
            }
            const auto version = cmd_read_version(id);
            if (!version) {
                return Expected<int>::failure(version.error());
            }
            return Expected<int>(version.value()[static_cast<size_t>(parsed - 1)]);
        }
        return Expected<int>::failure("Unknown parameter '" + key + "'");
    }

    Result UbtechDriver::write_param(const ServoId id, const std::string& key, const int value) {
        if (fd_ < 0) {
            return Result::failure("Serial port is not open");
        }
        if (key == "id") {
            if (value < kMinServoId || value > kMaxServoId) {
                return Result::failure("Servo ID out of range 1..240");
            }
            const auto result = cmd_set_id(id, static_cast<ServoId>(value));
            if (result) {
                // ID 修改立即生效：本地换算配置同步迁移。
                if (const auto it = configs_.find(id); it != configs_.end()) {
                    auto config = it->second;
                    config.id = static_cast<ServoId>(value);
                    configs_.erase(it);
                    configs_[config.id] = config;
                }
            }
            return result;
        }
        if (key == "angle_offset") {
            if (value < -90 || value > 90) {
                return Result::failure("Angle offset out of range -90..90 (1/3 deg)");
            }
            return cmd_set_offset(id, value);
        }
        if (key.rfind("firmware_v", 0) == 0) {
            return Result::failure("Parameter '" + key + "' is read-only");
        }
        return Result::failure("Unknown parameter '" + key + "'");
    }

    void UbtechDriver::register_driver() {
        servo_core::DriverRegistry::instance().add("ubtech", [] {
            return std::make_unique<UbtechDriver>();
        });
    }

} // namespace servo_backends
