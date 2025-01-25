from controller import RudderController

controller = RudderController(0.0, 0.0, 0, 1, 1, 0.1, 0.5)
print(controller.is_target_reached)
controller.reset_setpoint(-120.0, 0.0)
print(controller.is_target_reached)
while not controller.is_target_reached:
    controller.update_state()
