#ifndef BOATTEST_COMMON_H_
#define BOATTEST_COMMON_H_

/* Include Files */
#include <string>
#include <vector>

/* Objects */
typedef enum {
    ROS,
    CAN,
    NONE,
} testType;

class BoatTest
{
    std::string              name;
    testType                 type;
    int                      timeout_sec;
    std::vector<std::string> data;

public:
    BoatTest();
    BoatTest(std::string id, testType test_type, int timeout, std::vector<std::string> test_data);
    std::string              getName(BoatTest * test);
    testType                 getTestType(BoatTest * test);
    std::vector<std::string> getTestData(BoatTest * test);
    int                      getTimeout(BoatTest * test);
};

#endif
