#include "servo_core/registry.hpp"

#include <ranges>

namespace servo_core {

    DriverRegistry& DriverRegistry::instance() {
        static DriverRegistry registry;
        return registry;
    }

    void DriverRegistry::add(const std::string& name, Factory factory) {
        factories_[name] = std::move(factory);
    }

    std::unique_ptr<IServoDriver> DriverRegistry::create(const std::string& name) const {
        const auto it = factories_.find(name);
        if (it == factories_.end()) {
            return nullptr;
        }
        return it->second();
    }

    std::vector<std::string> DriverRegistry::names() const {
        std::vector<std::string> result;
        result.reserve(factories_.size());
        for (const auto& name : factories_ | std::views::keys) {
            result.push_back(name);
        }
        return result;
    }

} // namespace servo_core
