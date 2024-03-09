#include <gtest/gtest.h>

#include "cached_fib.h"

static constexpr int DEFAULT_SIZE = 5;
static CachedFib     g_test_fib   = CachedFib(DEFAULT_SIZE);

class TestFib : public ::testing::Test
{
protected:
    TestFib()
    {
        // Every time a test is started, testFib is reinitialized with a constructor parameter of 5
        g_test_fib = CachedFib(DEFAULT_SIZE);
    }

    ~TestFib() override
    {
        // Clean up after a test
    }
};

TEST_F(TestFib, TestBasic) { ASSERT_EQ(g_test_fib.getFib(5), 3) << "5th fibonacci number must be 3!"; }

TEST_F(TestFib, TestBasic2)
{
    ASSERT_EQ(g_test_fib.getFib(6), 5);
    ASSERT_EQ(g_test_fib.getFib(7), 8);
}
