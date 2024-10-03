#include </usr/include/ompl-1.6/ompl/base/OptimizationObjective.h>
#include </usr/include/ompl-1.6/ompl/base/Planner.h>
#include </usr/include/ompl-1.6/ompl/base/PlannerStatus.h>
#include </usr/include/ompl-1.6/ompl/base/ScopedState.h>
#include </usr/include/ompl-1.6/ompl/base/SpaceInformation.h>
#include </usr/include/ompl-1.6/ompl/base/State.h>
#include </usr/include/ompl-1.6/ompl/base/StateValidityChecker.h>
#include </usr/include/ompl-1.6/ompl/base/spaces/RealVectorBounds.h>
#include </usr/include/ompl-1.6/ompl/base/spaces/SE2StateSpace.h>
#include </usr/include/ompl-1.6/ompl/geometric/PathGeometric.h>
#include </usr/include/ompl-1.6/ompl/geometric/SimpleSetup.h>
#include </usr/include/ompl-1.6/ompl/geometric/planners/rrt/RRTstar.h>
#include </usr/include/ompl-1.6/ompl/util/Console.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_OMPL(py::module & m)
{
    // Binding for ompl::base::State
    py::class_<ompl::base::ScopedState<ompl::base::SE2StateSpace>>(m, "State")
      .def(py::init<ompl::base::StateSpacePtr>(), py::arg("space"));

    // Binding for ompl::base::RealVectorBounds
    py::class_<ompl::base::RealVectorBounds>(m, "RealVectorBounds")
      .def(py::init<unsigned int>(), py::arg("dim"))
      .def(
        "setLow", static_cast<void (ompl::base::RealVectorBounds::*)(unsigned int, double)>(
                    &ompl::base::RealVectorBounds::setLow))
      .def(
        "setHigh", static_cast<void (ompl::base::RealVectorBounds::*)(unsigned int, double)>(
                     &ompl::base::RealVectorBounds::setHigh))
      .def("low", [](const ompl::base::RealVectorBounds & self) { return self.low; })
      .def("high", [](const ompl::base::RealVectorBounds & self) { return self.high; })
      .def("check", &ompl::base::RealVectorBounds::check);

    // Binding for ompl::base::SE2StateSpace
    py::class_<ompl::base::SE2StateSpace>(m, "SE2StateSpace")
      .def(py::init<>())
      .def(
        "setBounds",
        [](ompl::base::SE2StateSpace & self, const ompl::base::RealVectorBounds & bounds) { self.setBounds(bounds); },
        py::arg("bounds"));

    // Binding for ompl::base::StateValidityCheckerFn
    py::class_<ompl::base::StateValidityCheckerFn>(m, "StateValidityCheckerFn").def(py::init<>());

    // Binding for ompl::geometric::RRTstar
    py::class_<ompl::geometric::RRTstar>(m, "RRTstar")
      .def(py::init<const ompl::base::SpaceInformationPtr &>(), py::arg("space_information"))
      .def("solve", &ompl::geometric::RRTstar::solve)
      .def("setRange", &ompl::geometric::RRTstar::setRange)
      .def("getRange", &ompl::geometric::RRTstar::getRange);

    // Binding for ompl::geometric::SimpleSetup
    py::class_<ompl::geometric::SimpleSetup>(m, "SimpleSetup")
      .def(py::init<const ompl::base::StateSpacePtr &>(), py::arg("space"))
      .def(
        "setStartAndGoalStates",
        static_cast<void (ompl::geometric::SimpleSetup::*)(
          const ompl::base::ScopedState<> &, const ompl::base::ScopedState<> &, double)>(
          &ompl::geometric::SimpleSetup::setStartAndGoalStates),
        py::arg("start"), py::arg("goal"), py::arg("threshold") = std::numeric_limits<double>::epsilon())
      .def(
        "solve",
        static_cast<ompl::base::PlannerStatus (ompl::geometric::SimpleSetup::*)(double)>(
          &ompl::geometric::SimpleSetup::solve),
        py::arg("time") = 1.0)
      .def(
        "getSolutionPath",
        static_cast<ompl::geometric::PathGeometric & (ompl::geometric::SimpleSetup::*)() const>(
          &ompl::geometric::SimpleSetup::getSolutionPath),
        "Get the solution path. Throws an exception if no solution is available.")
      .def(
        "setStateValidityChecker",
        static_cast<void (ompl::geometric::SimpleSetup::*)(const ompl::base::StateValidityCheckerFn &)>(
          &ompl::geometric::SimpleSetup::setStateValidityChecker),
        py::arg("svc"))
      .def(
        "getSpaceInformation",
        [](const ompl::geometric::SimpleSetup & self) {
            return self.getSpaceInformation();  // Use the correct return type
        },
        "Get the current instance of the space information.")
      .def(
        "setOptimizationObjective",
        static_cast<void (ompl::geometric::SimpleSetup::*)(const ompl::base::OptimizationObjectivePtr &)>(
          &ompl::geometric::SimpleSetup::setOptimizationObjective),
        py::arg("optimizationObjective"));
}

PYBIND11_MODULE(pyompl, m) { bind_OMPL(m); }
