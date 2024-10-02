#include </usr/include/ompl-1.6/ompl/util/Console.h>  // Include the Console header
#include <pybind11/pybind11.h>

namespace py = pybind11;

void greet()
{
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);  // Set the log level to LOG_DEV2
    OMPL_INFORM("Hello, world!");                 // Print the message "Hello, world!"
}

PYBIND11_MODULE(hello_module, m) { m.def("greet", &greet, "A function that prints 'Hello, world!'"); }
