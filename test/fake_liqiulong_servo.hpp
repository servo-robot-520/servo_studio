#pragma once

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace servo_backends::test {

    /// @brief 基于伪终端的 MicroDuck（UBTECH-LQL）总线模拟器。
    ///
    /// 按固件行为应答 SCS 帧：PING/READ/WRITE/SYNC_WRITE（双变体），
    /// REG_WRITE/ACTION/SYNC_READ（仅 SAMD10）；实现锁门（48=1 拒写 cfg，
    /// 40/33/41/48 例外）、应答开关（reg8=0 写命令无 ACK）、改 ID 回新 ID、
    /// mega8 的 49=0xA5 保存触发；可注入错误状态字节与坏校验和。
    class FakeLiqiulongBus {
    public:
        struct Servo {
            uint8_t id{1};
            bool samd10{true}; // false = mega8 变体
            std::array<uint8_t, 2> version{1, 11};
            std::array<uint8_t, 162> regs{};
            bool silent{false};
            uint8_t status_inject{0}; // 应答状态字节（模拟 g_err 错误位）
            bool corrupt{false};      // 破坏应答校验和

            // 观测（断言用）。
            int save_count{0};                // 49=0xA5 触发次数
            int regwrite_count{0};            // REG_WRITE 次数
            int action_count{0};              // ACTION 次数
            std::vector<std::vector<uint8_t>> writes; // 每次 WRITE 的 (addr, data…)
        };

        explicit FakeLiqiulongBus(std::vector<Servo> servos) :
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
            for (auto& servo : servos_) {
                init_regs(servo);
            }
            thread_ = std::thread([this] { run(); });
        }

        ~FakeLiqiulongBus() {
            stop_.store(true);
            if (thread_.joinable()) {
                thread_.join();
            }
            ::close(master_fd_);
        }

        FakeLiqiulongBus(const FakeLiqiulongBus&) = delete;
        FakeLiqiulongBus& operator=(const FakeLiqiulongBus&) = delete;

        const std::string& slave_path() const {
            return slave_path_;
        }

        Servo servo_state(const uint8_t id) const {
            std::lock_guard<std::mutex> lock(mutex_);
            const Servo* servo = find(id);
            if (servo == nullptr) {
                throw std::runtime_error("unknown servo id");
            }
            return *servo;
        }

        /// 注入反馈寄存器（模拟角度/速度/负载等只读块）。
        void set_regs(const uint8_t id, const uint8_t address,
                      const std::vector<uint8_t>& data) {
            std::lock_guard<std::mutex> lock(mutex_);
            Servo* servo = find(id);
            if (servo == nullptr) {
                throw std::runtime_error("unknown servo id");
            }
            for (size_t i = 0; i < data.size(); ++i) {
                servo->regs[static_cast<size_t>(address) + i] = data[i];
            }
        }

        /// SYNC_WRITE 观测（分批断言）。
        std::pair<int, int> sync_stats() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return {sync_frames_, sync_max_ids_};
        }

        std::vector<std::vector<uint8_t>> received_frames() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return received_;
        }

    private:
        static void init_regs(Servo& servo) {
            servo.regs[8] = 1;  // 应答开关
            servo.regs[48] = 1; // 配置锁
            servo.regs[40] = 1; // 扭矩使能
            servo.regs[5] = servo.id;
            servo.regs[3] = servo.version[0];
            servo.regs[4] = servo.version[1];
            servo.regs[6] = servo.samd10 ? 0 : 4; // 波特率码（mega8 恒 4）
            servo.regs[9] = servo.samd10 ? 0x02 : 0x00;  // 限位默认
            servo.regs[10] = servo.samd10 ? 0x1C : 0x18;
            servo.regs[11] = servo.samd10 ? 0x0E : 0x0F;
            servo.regs[12] = servo.samd10 ? 0xD8 : 0xF0;
        }

        /// 固件锁门涉及的 cfg 寄存器（40/33/41/48 豁免）。
        static bool is_cfg(const uint8_t address) {
            switch (address) {
                case 5:  case 6:  case 9:  case 10: case 11: case 12: case 13: case 16:
                case 17: case 21: case 22: case 23: case 24: case 26: case 27: case 29:
                case 73: case 74: case 75: case 76: case 82: case 83: case 84: case 85:
                case 86: case 87: case 88: case 89: case 37: case 71:
                    return true;
                default:
                    return false;
            }
        }

        Servo* find(const uint8_t id) {
            for (auto& servo : servos_) {
                if (servo.id == id) {
                    return &servo;
                }
            }
            return nullptr;
        }

        const Servo* find(const uint8_t id) const {
            for (const auto& servo : servos_) {
                if (servo.id == id) {
                    return &servo;
                }
            }
            return nullptr;
        }

        static uint8_t checksum(const uint8_t id, const uint8_t length, const uint8_t* body) {
            uint8_t sum = static_cast<uint8_t>(id + length);
            for (int i = 0; i < length - 1; ++i) {
                sum = static_cast<uint8_t>(sum + body[i]);
            }
            return static_cast<uint8_t>(~sum);
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

        void reply_frame(const Servo& servo, const std::vector<uint8_t>& params) {
            std::vector<uint8_t> body;
            body.push_back(servo.status_inject); // 状态字节
            body.insert(body.end(), params.begin(), params.end());
            body.push_back(0);
            const auto length = static_cast<uint8_t>(body.size());
            body.back() = checksum(servo.id, length, body.data());
            std::vector<uint8_t> frame = {0xFF, 0xFF, servo.id, length};
            if (servo.corrupt) {
                body.back() = static_cast<uint8_t>(body.back() + 1);
            }
            frame.insert(frame.end(), body.begin(), body.end());
            write_all(frame.data(), frame.size());
        }

        void sram_write(Servo& servo, const uint8_t address, const uint8_t value) {
            // 锁门：锁定时拒写 cfg 寄存器（40/33/41/48 例外），静默跳过。
            if (servo.regs[48] && is_cfg(address) && address != 40 && address != 33 &&
                address != 41 && address != 48) {
                return;
            }
            if (address >= servo.regs.size()) {
                return;
            }
            servo.regs[address] = value;
            if (address == 5) {
                servo.id = value; // 改 ID 立即生效
            }
            if (address == 49 && value == 0xA5) {
                servo.save_count++; // mega8 EEPROM 保存触发
            }
        }

        void handle_frame(const uint8_t* frame) {
            const uint8_t id = frame[2];
            const uint8_t instruction = frame[4];
            const uint8_t* params = frame + 5;
            const int params_len = frame[3] - 2; // LEN = INS+参数+校验和

            if (instruction == 0x83) { // SYNC_WRITE：按目标 ID 逐颗写入
                const uint8_t per = params[1];
                int ids_in_frame = 0;
                for (int i = 2; i + per < params_len + 1; i += per + 1) {
                    Servo* member = find(params[i]);
                    if (member == nullptr) {
                        continue;
                    }
                    ++ids_in_frame;
                    for (uint8_t k = 0; k < per; ++k) {
                        sram_write(*member, static_cast<uint8_t>(params[0] + k),
                                   params[i + 1 + k]);
                    }
                }
                sync_frames_++;
                sync_max_ids_ = std::max(sync_max_ids_, ids_in_frame);
                return;
            }

            if (id == 0xFE) { // 广播
                if (instruction == 0x05) { // ACTION 广播触发
                    for (auto& servo : servos_) {
                        if (servo.samd10) {
                            servo.action_count++;
                        }
                    }
                }
                // 广播 WRITE/READ/PING：本模拟器不作断言依赖，忽略。
                return;
            }

            Servo* found = find(id);
            if (found == nullptr || found->silent) {
                return;
            }
            Servo& servo = *found;

            switch (instruction) {
                case 0x01: // PING
                    reply_frame(servo, {});
                    return;
                case 0x02: { // READ
                    const uint8_t address = params[0];
                    const uint8_t length = params[1];
                    std::vector<uint8_t> data(length, 0);
                    const size_t regn = servo.samd10 ? 162 : 128;
                    for (uint8_t i = 0; i < length; ++i) {
                        if (static_cast<size_t>(address) + i < regn) {
                            data[i] = servo.regs[static_cast<size_t>(address) + i];
                        }
                    }
                    reply_frame(servo, data);
                    return;
                }
                case 0x03: { // WRITE：逐字节走锁门
                    servo.writes.emplace_back(params, params + params_len);
                    for (int i = 1; i < params_len; ++i) {
                        sram_write(servo, static_cast<uint8_t>(params[0] + i - 1), params[i]);
                    }
                    if (servo.regs[8] != 0) {
                        reply_frame(servo, {});
                    }
                    return;
                }
                case 0x04: { // REG_WRITE（仅 SAMD10）
                    if (!servo.samd10) {
                        return;
                    }
                    servo.regwrite_count++;
                    if (servo.regs[8] != 0) {
                        reply_frame(servo, {});
                    }
                    return;
                }
                case 0x05: // ACTION（仅 SAMD10，应答不受 reg8 约束）
                    if (!servo.samd10) {
                        return;
                    }
                    servo.action_count++;
                    reply_frame(servo, {});
                    return;
                default:
                    return;
            }
        }

        void run() {
            std::vector<uint8_t> buffer;
            while (!stop_.load()) {
                pollfd pfd{master_fd_, POLLIN, 0};
                if (::poll(&pfd, 1, 5) <= 0) {
                    continue;
                }
                uint8_t chunk[256];
                const ssize_t n = ::read(master_fd_, chunk, sizeof(chunk));
                if (n <= 0) {
                    continue;
                }
                buffer.insert(buffer.end(), chunk, chunk + n);
                while (buffer.size() >= 4) {
                    // 帧同步：找 FF FF，取 LEN 凑齐 LEN+4。
                    size_t start = buffer.size();
                    for (size_t i = 0; i + 1 < buffer.size(); ++i) {
                        if (buffer[i] == 0xFF && buffer[i + 1] == 0xFF) {
                            start = i;
                            break;
                        }
                    }
                    if (start > 0) {
                        buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(start));
                    }
                    if (buffer.size() < 4) {
                        break;
                    }
                    const size_t total = static_cast<size_t>(buffer[3]) + 4;
                    if (buffer.size() < total) {
                        break;
                    }
                    std::vector<uint8_t> frame(buffer.begin(),
                                               buffer.begin() + static_cast<long>(total));
                    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(total));
                    // 校验请求帧（坏帧丢弃）。
                    if (checksum(frame[2], frame[3], frame.data() + 4) != frame.back()) {
                        continue;
                    }
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        received_.push_back(frame);
                    }
                    std::lock_guard<std::mutex> lock(mutex_);
                    handle_frame(frame.data());
                }
            }
        }

        int master_fd_{-1};
        std::string slave_path_;
        std::vector<Servo> servos_;
        std::vector<std::vector<uint8_t>> received_;
        int sync_frames_{0};
        int sync_max_ids_{0};
        mutable std::mutex mutex_;
        std::atomic<bool> stop_{false};
        std::thread thread_;
    };

} // namespace servo_backends::test
