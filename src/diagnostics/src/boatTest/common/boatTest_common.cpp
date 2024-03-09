#include "boatTest_common.h"

BoatTest::BoatTest() { name = "NONESPECIFIED"; }
BoatTest::BoatTest(std::string id, testType test_type, int timeout, std::vector<std::string> test_data)
{
    name        = id;
    type        = test_type;
    timeout_sec = timeout;
    data        = test_data;
}

std::string              BoatTest::getName(BoatTest * test) { return test->name; }
testType                 BoatTest::getTestType(BoatTest * test) { return test->type; }
std::vector<std::string> BoatTest::getTestData(BoatTest * test) { return test->data; }
int                      BoatTest::getTimeout(BoatTest * test) { return test->timeout_sec; }
