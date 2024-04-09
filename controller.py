wind_active = True  # Select whether you want to activate wind or not
group_number = None  # Enter your group number here


# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor

    action = (0, 0)
    return action
