#pragma once

#include <string>
#include <utility>

namespace servo_core {

    /// @brief 极简结果类型：只表示成功，或在失败时携带一条错误信息。
    ///        与驱动侧的 Result 形态一致，但属于中间层自己的词汇，
    ///        不依赖任何厂商头文件。
    class Result {
    public:
        Result() = default;

        /// @brief 构造成功结果。
        static Result success() {
            return Result{};
        }

        /// @brief 构造失败结果并保存错误信息。
        /// @param message 错误描述。
        static Result failure(std::string message) {
            Result result;
            result.ok_ = false;
            result.message_ = std::move(message);
            return result;
        }

        /// @brief 是否成功。
        [[nodiscard]] bool has_value() const {
            return ok_;
        }

        /// @brief 是否成功，便于使用 if (result) 判断。
        explicit operator bool() const {
            return ok_;
        }

        /// @brief 获取错误信息；成功时为空字符串。
        [[nodiscard]] const std::string& error() const {
            return message_;
        }

    private:
        bool ok_{true};
        std::string message_;
    };

    /// @brief 极简 expected 类型：成功时携带值，失败时携带错误信息。
    template <typename T>
    class Expected {
    public:
        /// @brief 用成功值构造。
        explicit Expected(T value) :
            value_(std::move(value)) {
        }

        /// @brief 构造失败结果。
        /// @param message 错误描述。
        static Expected failure(std::string message) {
            Expected result{T{}};
            result.has_value_ = false;
            result.message_ = std::move(message);
            return result;
        }

        /// @brief 是否持有有效值。
        [[nodiscard]] bool has_value() const {
            return has_value_;
        }

        /// @brief 是否持有有效值，便于使用 if (expected) 判断。
        explicit operator bool() const {
            return has_value_;
        }

        /// @brief 获取值；仅在 has_value() 为 true 时有效。
        const T& value() const {
            return value_;
        }

        /// @brief 获取错误信息；成功时为空字符串。
        [[nodiscard]] const std::string& error() const {
            return message_;
        }

    private:
        T value_{};
        std::string message_;
        bool has_value_{true};
    };

} // namespace servo_core
