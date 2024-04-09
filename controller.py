wind_active = False  # Select whether you want to activate wind or not
group_number = 32  # Enter your group number here
prev_error = None
i_u = 0
min_error = 0
max_error = 0
start_timer = False
timer = 0
osilationReached = False

# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor
    global prev_error
    global i_u
    global min_error
    global max_error
    global start_timer
    global timer
    global osilationReached
    
    if prev_error is None:
        prev_error = state[0] - target_pos[0] + state[1] - target_pos[1]
    
    # Constants
    Kp = 0.292085 # reduce error
    Kd = 0 # reduce overshoot
    Ki = 0 # reduce steady state error
    

    # Calculate the error (target position - current position)
    error = state[0] - target_pos[0] + state[1] - target_pos[1]
    
    # Calculate the proportional, integral, and derivative terms
    
    # Proportional term
    p_u = Kp * error
    
    # Integral term
    i_u += Ki * error * dt
    
    # Derivative term
    d_u = Kd * (error - prev_error) / dt
    
    pid_u = p_u + i_u + d_u
    
    action = [pid_u, pid_u]
    
    prev_error = error
    
    return action


# Get the Ku and Tu values, debgging purposes
def ku_and_tu(error, dt):
    if error > min_error:
        min_error = error
        print("Dif: ", max_error + min_error)
        
    if error < max_error:
        max_error = error
        print("Dif: ", max_error + min_error)
    
    
    timer += dt
    if osilationReached == False and -0.0001 <= (max_error + min_error) <= 0.0001 and timer > 10:
        print("Osilation Reached")
        osilationReached = True
        

    # If the osilation is reached, calculate the time of one cycle
    if osilationReached == True:
        # If the error is the same as the min error, start the timer
        if min_error - 0.01 <= error <= min_error + 0.01:
            timer = 0
            print("Start Timer")
            start_timer = True
    
        # If the error is the same as the max error, stop the timer and print the time
        if max_error - 0.01 <= error <= max_error + 0.01:
            print("TU one cycle is: ", timer)
            osilationReached = False
