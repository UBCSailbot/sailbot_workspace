#pragma once

#include <string>
#include <vector>

constexpr auto CACHED_FIB_TOPIC_IN  = "cached_fib_in";
constexpr auto CACHED_FIB_TOPIC_OUT = "cached_fib_out";

class CachedFib
{
private:
    std::vector<int> cache_;

public:
    explicit CachedFib(std::size_t);
    int getFib(std::size_t);
};
