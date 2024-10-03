import pyompl as po


# Create a state space
def create_state_space():
    state_space = po.StateSpace()
    assert state_space is not None
    return state_space


def create_bounds():
    bounds = po.RealVectorBounds(dim=2)
    assert bounds is not None
    return bounds


def modify_and_check_bounds(bounds):
    bounds.setLow(index=0, value=0)
    bounds.setLow(index=1, value=0)
    bounds.setHigh(index=0, value=1)
    bounds.setHigh(index=1, value=1)
    print(
        "state space bounds: "
        f"x=[{bounds.low[0]}, {bounds.high[0]}]; "
        f"y=[{bounds.low[1]}, {bounds.high[1]}]"
    )
    bounds.check()


def set_space_bounds():
    bounds = create_bounds()
    modify_and_check_bounds(bounds)
    space = create_state_space()
    space.setBounds(bounds)


def create_simple_setup():
    space = create_state_space()
    simple_setup = po.SimpleSetup(space)


if __name__ == "main":
    create_state_space()
