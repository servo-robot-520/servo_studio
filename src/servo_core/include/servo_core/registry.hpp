#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "servo_core/driver.hpp"

namespace servo_core {

    /// @brief 驱动注册表：驱动名 → 工厂。后端在静态初始化或显式注册时登记。
    class DriverRegistry {
    public:
        using Factory = std::function<std::unique_ptr<IServoDriver>()>;

        /// @brief 获取全局实例。
        static DriverRegistry& instance();

        /// @brief 注册驱动工厂；同名覆盖。
        void add(const std::string& name, Factory factory);

        /// @brief 创建驱动实例。
        /// @return 未注册的名称返回 nullptr。
        [[nodiscard]] std::unique_ptr<IServoDriver> create(const std::string& name) const;

        /// @brief 已注册的驱动名列表。
        [[nodiscard]] std::vector<std::string> names() const;

    private:
        std::map<std::string, Factory> factories_;
    };

} // namespace servo_core
