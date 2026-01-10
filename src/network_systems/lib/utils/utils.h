#pragma once

#include <boost/math/special_functions/next.hpp>
#include <concepts>
#include <iostream>
#include <optional>
#include <sstream>

// Define a concept for arithmetic types
template <typename T>
concept arithmetic = std::integral<T> or std::floating_point<T>;
template <typename T>
concept not_float = not std::floating_point<T>;

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

/**
 * @brief Calculate floating point equality using the GoogleTest default definition
 *        http://google.github.io/googletest/reference/assertions.html#floating-point
 *
 * @tparam T       A floating point type (float, double, etc)
 * @param to_check Value to compare to expected
 * @param expected Expected value
 * @param err_msg  String that gets printed to stderr on failure
 * @return true if "to_check" is close enough to "expected", false otherwise
 */
template <std::floating_point T>
bool isFloatEQ(T to_check, T expected, const std::string & err_msg)
{
    constexpr int ALLOWED_ULP_DIFF = 4;

    int diff = boost::math::float_distance(to_check, expected);
    if (std::abs(diff) <= ALLOWED_ULP_DIFF) {
        return true;
    }
    if (!err_msg.empty()) {
        std::cerr << err_msg << std::endl;
    }
    return false;
}

/**
 * @brief Calls default isFloatEq<T>(T, T, string) with an empty error string
 *
 */
template <std::floating_point T>
bool isFloatEQ(T to_check, T expected)
{
    return isFloatEQ<T>(to_check, expected, "");
}

/**
 * @brief Check if two non-floating point values are equal and output an error on mismatch
 *
 * @tparam T non-floating point type
 * @param rcvd     received value
 * @param expected expected value
 * @param err_msg  error string
 * @return true  if rcvd and expected match
 * @return false otherwise
 */
template <not_float T>
bool checkEQ(T rcvd, T expected, const std::string & err_msg)
{
    if (rcvd != expected) {
        std::cerr << "Expected: " << expected << " but received: " << rcvd << std::endl;
        std::cerr << err_msg << std::endl;
        return false;
    }
    return true;
};

/**
 * @brief Check if two floating point values are equal using isFloatEq() and output an error on mismatch
 *
 * @tparam T floating point values (float, double, etc...)
 * @param rcvd     received value
 * @param expected expected value
 * @param err_msg  error string
 * @return true  if rcvd and expected match
 * @return false otherwise
 */
template <std::floating_point T>
bool checkEQ(T rcvd, T expected, const std::string & err_msg)
{
    std::stringstream ss;
    ss << "Expected: " << expected << " but received: " << rcvd << "\n" << err_msg;
    return static_cast<bool>(utils::isFloatEQ<T>(rcvd, expected, ss.str()));
};

/**
 * @brief Generic function that takes angle in range [0, 360) and bounds them  (-180 <= 180]
 *
 * @tparam Type Generic type
 * @param angle Input angle (guaranteed to be [0,360))
 * @return auto Returns the same type as the input if signed, otherwise returns the signed
           type of the same bit width
*/
template <typename Type>
static auto boundTo180(Type angle)
{
    if constexpr (std::is_integral_v<Type> && std::is_unsigned_v<Type>) {
        using ReturnType      = std::make_signed_t<Type>;
        ReturnType angle_copy = static_cast<ReturnType>(angle);
        if (angle_copy > 180) {  //NOLINT(readability-magic-numbers)
            angle_copy -= 360;   //NOLINT(readability-magic-numbers)
        }
        return angle_copy;
    } else {
        // signed integer or floating point
        if (angle > 180) {  //NOLINT(readability-magic-numbers)
            angle -= 360;   //NOLINT(readability-magic-numbers)
        }
        return angle;
    }
}

/**
 * @brief Generic function that takes angles and bounds them to [0, 360)
 *
 * @tparam Type Generic type
 * @param angle Input angle (note: the angle is not guaranteed by CAN transceiver to be (-180, 180].
                We assume this by definition of HelperHeading.msg
 * @return auto Returns the same type as the input if signed, otherwise returns the signed
                type of the same bit width
*/
template <typename Type>
static auto boundTo360(Type angle)
{
    // if unsigned int
    if constexpr (std::is_integral_v<Type> && std::is_unsigned_v<Type>) {
        using ReturnType      = std::make_signed_t<Type>;
        ReturnType angle_copy = static_cast<ReturnType>(angle);
        if (angle_copy < 0) {
            angle_copy += 360;  //NOLINT(readability-magic-numbers)
        }
        return angle_copy;
    } else {
        // is signed int or float
        if (angle < 0) {
            angle += 360;  //NOLINT(readability-magic-numbers)
        }
        return angle;
    }
}

/**
 * @brief Simple class to count number of failures
 *
 */
class FailTracker
{
public:
    /**
     * @brief Update the tracker
     *
     * @param was_success result of operation to check
     */
    void track(bool was_success)
    {
        if (!was_success) {
            fail_count_++;
        }
    }

    /**
     * @brief Reset fail count
     *
     */
    void reset() { fail_count_ = 0; }

    /**
     * @return true  if any failures were tracked
     * @return false otherwise
     */
    bool failed() const { return fail_count_ != 0; }

    /**
     * @return number of failures
     */
    uint32_t failCount() const { return fail_count_; }

private:
    uint32_t fail_count_ = 0;
};

}  // namespace utils
