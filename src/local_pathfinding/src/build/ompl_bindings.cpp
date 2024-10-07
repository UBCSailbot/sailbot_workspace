#include </usr/include/ompl-1.6/ompl/base/Cost.h>
#include </usr/include/ompl-1.6/ompl/base/Goal.h>
#include </usr/include/ompl-1.6/ompl/base/OptimizationObjective.h>
#include </usr/include/ompl-1.6/ompl/base/Planner.h>
#include </usr/include/ompl-1.6/ompl/base/PlannerStatus.h>
#include </usr/include/ompl-1.6/ompl/base/ScopedState.h>
#include </usr/include/ompl-1.6/ompl/base/SpaceInformation.h>
#include </usr/include/ompl-1.6/ompl/base/State.h>
#include </usr/include/ompl-1.6/ompl/base/StateSpace.h>
#include </usr/include/ompl-1.6/ompl/base/StateValidityChecker.h>
#include </usr/include/ompl-1.6/ompl/base/goals/GoalState.h>
#include </usr/include/ompl-1.6/ompl/base/objectives/PathLengthOptimizationObjective.h>
#include </usr/include/ompl-1.6/ompl/base/objectives/StateCostIntegralObjective.h>
#include </usr/include/ompl-1.6/ompl/base/spaces/RealVectorBounds.h>
#include </usr/include/ompl-1.6/ompl/base/spaces/SE2StateSpace.h>
#include </usr/include/ompl-1.6/ompl/geometric/PathGeometric.h>
#include </usr/include/ompl-1.6/ompl/geometric/SimpleSetup.h>
#include </usr/include/ompl-1.6/ompl/geometric/planners/rrt/RRTstar.h>
#include </usr/include/ompl-1.6/ompl/util/Console.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

class StateValidityCheckerFn
{
public:
    using CheckerFunction = std::function<bool(const ompl::base::State *)>;

    StateValidityCheckerFn(CheckerFunction checker) : checker_(checker) {}

    bool isValid(const ompl::base::State * state) { return checker_(state); }

private:
    CheckerFunction checker_;
};

void bind_OMPL(py::module & m)
{
    // ########################################## OMPL PATH BINDINGS ###################################################
    py::class_<ompl::base::ScopedState<ompl::base::SE2StateSpace>>(m, "ScopedState")
      .def(py::init<ompl::base::StateSpacePtr>(), py::arg("space"))
      .def(
        "setXY",
        [](ompl::base::ScopedState<ompl::base::SE2StateSpace> & state, double x, double y) { state()->setXY(x, y); })
      .def("setY", [](ompl::base::ScopedState<ompl::base::SE2StateSpace> & state, double y) { state()->setY(y); })
      .def("setX", [](ompl::base::ScopedState<ompl::base::SE2StateSpace> & state, double x) { state()->setX(x); })
      .def("getX", [](const ompl::base::ScopedState<ompl::base::SE2StateSpace> & state) { return state()->getX(); })
      .def("getY", [](const ompl::base::ScopedState<ompl::base::SE2StateSpace> & state) { return state()->getY(); });

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

    py::class_<ompl::base::StateSpace, std::shared_ptr<ompl::base::StateSpace>>(m, "StateSpace");

    py::class_<ompl::base::SE2StateSpace, ompl::base::StateSpace, std::shared_ptr<ompl::base::SE2StateSpace>>(
      m, "SE2StateSpace")
      .def(py::init<>())
      .def(
        "setBounds",
        [](ompl::base::SE2StateSpace & self, const ompl::base::RealVectorBounds & bounds) { self.setBounds(bounds); },
        py::arg("bounds"));

    py::class_<ompl::base::SE2StateSpace::StateType>(m, "SE2StateSpaceStateType")
      .def(py::init<>())
      .def("setX", &ompl::base::SE2StateSpace::StateType::setX)
      .def("setY", &ompl::base::SE2StateSpace::StateType::setY)
      .def("setXY", &ompl::base::SE2StateSpace::StateType::setXY)
      .def("getX", &ompl::base::SE2StateSpace::StateType::getX)
      .def("getY", &ompl::base::SE2StateSpace::StateType::getY);

    py::class_<StateValidityCheckerFn>(m, "StateValidityCheckerFn")
      .def(
        py::init<StateValidityCheckerFn::CheckerFunction>(),
        "Constructs a StateValidityCheckerFn with a Python callable.")
      .def("is_valid", &StateValidityCheckerFn::isValid, "Checks if the state is valid.");

    py::class_<ompl::geometric::RRTstar>(m, "RRTstar")
      .def(py::init<const ompl::base::SpaceInformationPtr &>(), py::arg("space_information"))
      .def("solve", &ompl::geometric::RRTstar::solve)
      .def("setRange", &ompl::geometric::RRTstar::setRange)
      .def("getRange", &ompl::geometric::RRTstar::getRange);

    py::class_<ompl::base::SpaceInformation, std::shared_ptr<ompl::base::SpaceInformation>>(m, "SpaceInformation")
      .def(py::init<const ompl::base::StateSpacePtr &>(), py::arg("space"));

    py::class_<ompl::geometric::SimpleSetup>(m, "SimpleSetup")
      .def(py::init<const ompl::base::StateSpacePtr &>(), py::arg("space"))
      .def(
        "setStartAndGoalStatesSE2",
        [](
          py::object simple_setup, const ompl::base::ScopedState<ompl::base::SE2StateSpace> & start,
          const ompl::base::ScopedState<ompl::base::SE2StateSpace> & goal) {
            simple_setup.cast<ompl::geometric::SimpleSetup>().setStartAndGoalStates(
              ompl::base::ScopedState<>(start), ompl::base::ScopedState<>(goal));
        },
        py::arg("start"), py::arg("goal"))
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
        py::arg("svc"), "Sets a state validity checker.")
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
        py::arg("optimizationObjective"))
      .def(
        "setPlanner",
        static_cast<void (ompl::geometric::SimpleSetup::*)(const ompl::base::PlannerPtr &)>(
          &ompl::geometric::SimpleSetup::setPlanner),
        py::arg("planner"))
      .def("getGoal", &ompl::geometric::SimpleSetup::getGoal);

    // ########################################## OBJECTIVES BINDINGS ##################################################
    py::class_<ompl::base::OptimizationObjective, std::shared_ptr<ompl::base::OptimizationObjective>>(
      m, "OptimizationObjective");

    py::class_<
      ompl::base::MultiOptimizationObjective, ompl::base::OptimizationObjective,
      std::shared_ptr<ompl::base::MultiOptimizationObjective>>(m, "MultiOptimizationObjective")
      .def(py::init<const ompl::base::SpaceInformationPtr &>(), py::arg("si"))
      .def(
        "addObjective", &ompl::base::MultiOptimizationObjective::addObjective, py::arg("objective"), py::arg("weight"));

    py::class_<
      ompl::base::PathLengthOptimizationObjective, ompl::base::OptimizationObjective,
      std::shared_ptr<ompl::base::PathLengthOptimizationObjective>>(m, "PathLengthOptimizationObjective")
      .def(py::init<const ompl::base::SpaceInformationPtr &>(), py::arg("si"));

    py::class_<
      ompl::base::StateCostIntegralObjective, ompl::base::OptimizationObjective,
      std::shared_ptr<ompl::base::StateCostIntegralObjective>>(m, "StateCostIntegralObjective")
      .def(
        py::init<const ompl::base::SpaceInformationPtr &, bool>(), py::arg("si"),
        py::arg("enableMotionCostInterpolation") = false);

    py::class_<ompl::base::Cost>(m, "Cost").def(py::init<double>());
    py::class_<ompl::base::GoalState, std::shared_ptr<ompl::base::GoalState>>(m, "GoalState")
      .def(py::init<const ompl::base::SpaceInformationPtr &>(), py::arg("si"))
      .def("getState", [](ompl::base::GoalState & self) {
          ompl::base::State * state = self.getState();
          auto                space = self.getSpaceInformation()->getStateSpace();
          return ompl::base::ScopedState<ompl::base::SE2StateSpace>(space, state);
      });
    py::class_<ompl::base::Goal, std::shared_ptr<ompl::base::Goal>>(m, "Goal");
}

PYBIND11_MODULE(pyompl, m) { bind_OMPL(m); }
