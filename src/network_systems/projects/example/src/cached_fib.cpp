#include "cached_fib.h"

#include <iostream>
#include <vector>

CachedFib::CachedFib(const std::size_t n)
{
    cache_.push_back(0);
    cache_.push_back(1);
    for (std::size_t i = 2; i < n; i++) {
        cache_.push_back(cache_[i - 1] + cache_[i - 2]);
    }
}

int CachedFib::getFib(const std::size_t n)
{
    if (this->cache_.size() < n) {
        for (std::size_t i = cache_.size(); i < n; i++) {
            cache_.push_back(cache_[i - 1] + cache_[i - 2]);
        }
    }
    std::cout << cache_[n - 1] << std::endl;
    return cache_[n - 1];
}
