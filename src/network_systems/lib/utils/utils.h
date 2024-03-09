#pragma once

#include <concepts>
#include <optional>
#include <sstream>

// Define a concept for arithmetic types
template <typename T>
concept arithmetic = std::integral<T> or std::floating_point<T>;

namespace utils
{

/**
 * @brief Check if an input value is within its bounds
 *
 * @tparam T   arithmetic type to check
 * @param val  value to check
 * @param lbnd lower bound
 * @param ubnd upper bound
 * @return error string if out of bounds, std::nullopt if okay
 */
template <arithmetic T>
std::optional<std::string> isOutOfBounds(T val, T lbnd, T ubnd)
{
    if (val < lbnd || val > ubnd) {
        std::stringstream ss;
        ss << typeid(T).name() << "(" << val << ") is out of bounds" << std::endl
           << "lbnd: " << lbnd << std::endl
           << "ubnd: " << ubnd << std::endl;
        return ss.str();
    }
    return std::nullopt;
}

}  // namespace utils
