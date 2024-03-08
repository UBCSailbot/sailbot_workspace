"""Decorator functions used in the physics engine."""

from typing import Callable


# TODO Pull all subscriptions at once rather than one at a time to avoid needing to update in the
# future if more subscriptions are added
def require_all_subs_active(func: Callable):
    """A decorator that asserts all subscriptions must be active in a node in order for a the
    wrapped function to be executed. This decorator is only meant to be used inside the
    `PhysicsEngineNode` class.

    Args:
        func (Callable): The wrapped function.
    """

    def is_all_subs_active(obj) -> bool:
        is_desired_heading_valid = obj.desired_heading is not None
        obj.get_logger().debug(
            f"Is `desired_heading` subscription valid? {is_desired_heading_valid}"
        )
        return is_desired_heading_valid

    def check(obj, *args, **kwargs):
        if is_all_subs_active(obj):
            return func(obj, *args, **kwargs)
        else:
            obj.get_logger().warn(f"All subscribers must be active to invoke {func.__name__}")
            return

    return check
