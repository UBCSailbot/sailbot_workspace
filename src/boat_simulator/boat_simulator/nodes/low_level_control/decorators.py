"""Decorator functions used in the low level control node."""

from typing import Callable

from custom_interfaces.action import SimRudderActuation, SimSailTrimTabActuation


# TODO Devise a better method of making action server callacks mutually exclusive
class MutuallyExclusiveActionRoutine:
    """A decorator that prevents multiple instances of an action server routine executing at once,
    making it mutually exclusive.

    This decorator is only meant to be used inside the `LowLevelControlNode` class.
    """

    def __init__(self, action_type):
        self.__action_type = action_type

    def __call__(self, func: Callable):
        def check(obj, *args, **kwargs):
            if self.__is_action_active(obj, func):
                goal_handle = args[0]
                return self.__cancel_goal_request(obj, goal_handle)
            else:
                return self.__execute_action_routine(obj, func, *args, **kwargs)

        return check

    def __is_action_active(self, obj, func: Callable):
        if self.__action_type == SimRudderActuation:
            return obj.is_rudder_action_active
        elif self.__action_type == SimSailTrimTabActuation:
            return obj.is_sail_action_active
        else:
            obj.get_logger().error(
                f"Invalid action type {self.__action_type} for function {func.__name__}"
            )
            return True

    def __cancel_goal_request(self, obj, goal_handle):
        obj.get_logger().debug(
            f"An action of type {self.__action_type} is already active. Cancelling goal request"
        )
        goal_handle.abort()

    def __execute_action_routine(self, obj, func, *args, **kwargs):
        self.__set_active_flag(obj)

        try:
            result = func(obj, *args, **kwargs)
        except RuntimeError:
            obj.get_logger().error(f"An unexpected error occurred in {func.__name__}")
            result = None

        self.__unset_active_flag(obj)
        return result

    def __set_active_flag(self, obj):
        if self.__action_type == SimRudderActuation:
            obj._is_rudder_action_active = True
        elif self.__action_type == SimSailTrimTabActuation:
            obj._is_sail_action_active = True
        else:
            obj.get_logger().error(
                f"Invalid action type {self.__action_type} while setting action active flag"
            )

    def __unset_active_flag(self, obj):
        if self.__action_type == SimRudderActuation:
            obj._is_rudder_action_active = False
        elif self.__action_type == SimSailTrimTabActuation:
            obj._is_sail_action_active = False
        else:
            obj.get_logger().error(
                f"Invalid action type {self.__action_type} while unsetting action active flag"
            )
