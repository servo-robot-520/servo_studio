#pragma once

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace servo_backends::test {

    /// @brief 基于伪终端的 UBTECH 总线模拟器。
    ///
    /// 驱动打开 pty 从端，本类在主端按协议应答：转动/中止（短回应 0xAA）、
    /// 角度回读（完整帧）、修改 ID、偏移读写、固件版本读取。
    /// 可配置静默（模拟离线/锁定中）、EE 拒绝、AA+ID 短回应形态与坏校验和。
    class FakeUbtechBus {
    public:
        struct Servo {
            uint8_t id{1};
            int target_angle{120}; // 度
            int actual_angle{120}; // 度
            int offset{0};         // 1/3 度
            std::array<uint8_t, 4> version{1, 2, 3, 4};
            bool silent{false};     // 不回应任何命令
            bool ack_with_id{false}; // 短回应用 AA+ID 两字节形态
            bool reject{false};     // 改 ID / 写偏移回 EE
            // 最近一次 0x01 转动命令的原始参数（断言用）。
            int last_angle{-1};
            int last_move_units{-1};
            int last_lock_units{-1};
            bool stopped{false};
        };

        explicit FakeUbtechBus(std::vector<Servo> servos) :
            servos_(std::move(servos)) {
            master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
            if (master_fd_ < 0) {
                throw std::runtime_error("posix_openpt failed");
            }
            if (::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
                ::close(master_fd_);
                throw std::runtime_error("grantpt/unlockpt failed");
            }
            std::array<char, 128> path{};
            if (::ptsname_r(master_fd_, path.data(), path.size()) != 0) {
                ::close(master_fd_);
                throw std::runtime_error("ptsname_r failed");
            }
            slave_path_ = path.data();
            ::fcntl(master_fd_, F_SETFL, O_NONBLOCK);
            thread_ = std::thread([this] { run(); });
        }

        ~FakeUbtechBus() {
            stop_.store(true);
            if (thread_.joinable()) {
                thread_.join();
            }
            ::close(master_fd_);
        }

        FakeUbtechBus(const FakeUbtechBus&) = delete;
        FakeUbtechBus& operator=(const FakeUbtechBus&) = delete;

        const std::string& slave_path() const {
            return slave_path_;
        }

        /// @brief 取模拟舵机快照（断言收到的命令参数）。
        Servo servo_state(const uint8_t id) const {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& servo : servos_) {
                if (servo.id == id) {
                    return servo;
                }
            }
            throw std::runtime_error("unknown servo id");
        }

        /// @brief 取模拟舵机可变引用前的配置入口（构造参数即可，无需运行时改）。
        void set_corrupt_response(const bool corrupt) {
            std::lock_guard<std::mutex> lock(mutex_);
            corrupt_response_ = corrupt;
        }

        /// @brief 所有收到的完整命令帧（逐字节断言用）。
        std::vector<std::vector<uint8_t>> received_frames() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return received_;
        }

    private:
        static uint8_t checksum(const uint8_t* frame) {
            uint8_t sum = 0;
            for (int i = 2; i <= 7; ++i) {
                sum = static_cast<uint8_t>(sum + frame[i]);
            }
            return sum;
        }

        void write_all(const uint8_t* data, const size_t size) const {
            size_t written = 0;
            while (written < size) {
                const ssize_t n = ::write(master_fd_, data + written, size - written);
                if (n <= 0) {
                    return;
                }
                written += static_cast<size_t>(n);
            }
        }

        /// 短回应：0xAA [+ID]。
        void reply_ack(const Servo& servo) {
            const uint8_t bytes[2] = {0xAA, servo.id};
            write_all(bytes, servo.ack_with_id ? 2 : 1);
        }

        /// 完整回应帧（可选破坏校验和）。
        void reply_frame(std::array<uint8_t, 10> frame, const bool corrupt) {
            frame[8] = checksum(frame.data());
            if (corrupt) {
                frame[8] = static_cast<uint8_t>(frame[8] + 1);
            }
            write_all(frame.data(), frame.size());
        }

        void handle_frame(const uint8_t* frame) {
            const uint8_t h1 = frame[0];
            const uint8_t id = frame[2];
            const uint8_t cmd = frame[3];
            Servo* servo = nullptr;
            for (auto& candidate : servos_) {
                if (candidate.id == id) {
                    servo = &candidate;
                    break;
                }
            }
            if (servo == nullptr || servo->silent) {
                return; // 不在线/静默：不回应（协议规定失败也不回应）。
            }
            const bool corrupt = corrupt_response_;
            if (h1 == 0xFA) {
                switch (cmd) {
                    case 0x01: {
                        if (frame[4] == 0xFF) {
                            servo->stopped = true;
                        }
                        else {
                            servo->last_angle = frame[4];
                            servo->last_move_units = frame[5];
                            servo->last_lock_units = (frame[6] << 8) | frame[7];
                            servo->target_angle = servo->last_angle;
                        }
                        reply_ack(*servo);
                        break;
                    }
                    case 0x02: {
                        std::array<uint8_t, 10> response = {0xFA, 0xAF, id, 0x00, 0, 0, 0, 0, 0,
                                                            0xED};
                        response[4] = static_cast<uint8_t>(servo->target_angle >> 8);
                        response[5] = static_cast<uint8_t>(servo->target_angle & 0xFF);
                        response[6] = static_cast<uint8_t>(servo->actual_angle >> 8);
                        response[7] = static_cast<uint8_t>(servo->actual_angle & 0xFF);
                        reply_frame(response, corrupt);
                        break;
                    }
                    case 0xCD: {
                        const uint8_t new_id = frame[5];
                        std::array<uint8_t, 10> response = {
                            0xFA, 0xAF, id, static_cast<uint8_t>(servo->reject ? 0xEE : 0xAA),
                            0x00, servo->id, 0x00, 0x00, 0, 0xED};
                        if (!servo->reject) {
                            response[2] = new_id; // 回应中的 ID 已是修改后的新 ID。
                            servo->id = new_id;
                        }
                        reply_frame(response, corrupt);
                        break;
                    }
                    case 0xD2: {
                        servo->offset = static_cast<int>(
                            static_cast<int16_t>((frame[6] << 8) | frame[7]));
                        std::array<uint8_t, 10> response = {
                            0xFA, 0xAF, id, static_cast<uint8_t>(servo->reject ? 0xEE : 0xAA),
                            0x00, 0x00, 0x00, 0x00, 0, 0xED};
                        reply_frame(response, corrupt);
                        break;
                    }
                    case 0xD4: {
                        const auto raw = static_cast<uint16_t>(servo->offset);
                        // 字节 4-5 按协议为不确定值：故意填噪声验证容错。
                        std::array<uint8_t, 10> response = {0xFA, 0xAF, id, 0x00,
                                                            0xDE, 0xAD, 0, 0, 0, 0xED};
                        response[6] = static_cast<uint8_t>(raw >> 8);
                        response[7] = static_cast<uint8_t>(raw & 0xFF);
                        reply_frame(response, corrupt);
                        break;
                    }
                    default:
                        break;
                }
                return;
            }
            if (h1 == 0xFC && cmd == 0x01) {
                std::array<uint8_t, 10> response = {0xFC, 0xCF, id, 0x00,
                                                    servo->version[0], servo->version[1],
                                                    servo->version[2], servo->version[3], 0,
                                                    0xED};
                reply_frame(response, corrupt);
            }
        }

        void run() {
            std::vector<uint8_t> buffer;
            while (!stop_.load()) {
                pollfd pfd{master_fd_, POLLIN, 0};
                if (::poll(&pfd, 1, 5) <= 0) {
                    continue;
                }
                uint8_t chunk[64];
                const ssize_t n = ::read(master_fd_, chunk, sizeof(chunk));
                if (n <= 0) {
                    continue;
                }
                buffer.insert(buffer.end(), chunk, chunk + n);
                // 解析：找帧头凑满 10 字节；帧尾/校验和不对也消费掉（测试不投毒请求）。
                while (buffer.size() >= 10) {
                    size_t start = buffer.size();
                    for (size_t i = 0; i + 1 < buffer.size(); ++i) {
                        if ((buffer[i] == 0xFA && buffer[i + 1] == 0xAF) ||
                            (buffer[i] == 0xFC && buffer[i + 1] == 0xCF)) {
                            start = i;
                            break;
                        }
                    }
                    if (start > 0) {
                        buffer.erase(buffer.begin(),
                                     buffer.begin() + static_cast<long>(start));
                    }
                    if (buffer.size() < 10) {
                        break;
                    }
                    std::array<uint8_t, 10> frame{};
                    std::copy(buffer.begin(), buffer.begin() + 10, frame.begin());
                    buffer.erase(buffer.begin(), buffer.begin() + 10);
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        received_.push_back(std::vector<uint8_t>(frame.begin(), frame.end()));
                    }
                    handle_frame(frame.data());
                }
            }
        }

        int master_fd_{-1};
        std::string slave_path_;
        std::vector<Servo> servos_;
        std::vector<std::vector<uint8_t>> received_;
        bool corrupt_response_{false};
        mutable std::mutex mutex_;
        std::atomic<bool> stop_{false};
        std::thread thread_;
    };

} // namespace servo_backends::test
