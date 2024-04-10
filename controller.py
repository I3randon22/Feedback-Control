wind_active = False  # Select whether you want to activate wind or not
group_number = 32  # Enter your group number here
prev_error = None
i_u = 0
#0.292 posy

# Global state for tuning
tuning_state = {
    'kp': 0.29,  # Starting Kp
    'min_error': float('inf'),
    'max_error': float('-inf'),
    'oscillation_detected': False,
    'start_time': None,
    'timer': 0,
    'start_timer': False,
    'error_range': 0.01,  # Range within which we consider the error to be close to min or max
    'error_mag': 0.002,  # Error magnitude to consider oscillation,
    'error_diff': 0.1,  # Error difference to adjust Kp
    'process_time': 20  # Time to adjust Kp based on error difference
    
    
}

# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor
    global prev_error
    global tuning_state
    global i_u
    
    # Calculate the error (Change accorindly to your needs)
    error = state[1] - target_pos[1]
    
    # Set prev error for first iteration
    if prev_error is None:
        prev_error = error
        
    # Update min and max errors
    tuning_state['min_error'] = min(tuning_state['min_error'], error)
    tuning_state['max_error'] = max(tuning_state['max_error'], error)
    tuning_state['error_diff'] = tuning_state['max_error'] + tuning_state['min_error']
    
    # Constants when not testing
    Kp = tuning_state['kp']
    Kd = 0 # reduce overshoot
    Ki = 0 # reduce steady state error
    
    # Calculate the proportional, integral, and derivative terms
        
    # Proportional term
    p_u = Kp * error
    
    # Integral term
    i_u += Ki * error * dt
    
    # Derivative term
    d_u = Kd * (error - prev_error) / dt
    
    pid_u = p_u + i_u + d_u
    
    action = [pid_u, pid_u]
    
    # Increment timer
    tuning_state['timer'] += dt
    
    # Check for oscillation if not already detected
    if not tuning_state['oscillation_detected']:
        check_for_oscillation(error, dt)
    
    prev_error = error
    
    return action


def check_for_oscillation(error, dt):
    global tuning_state

    if tuning_state["timer"] > 10 and tuning_state['error_diff'] <= tuning_state['error_mag']:
        # If within error range of min or max error, start or stop timer
        if tuning_state['min_error'] - tuning_state['error_range'] <= error <= tuning_state['min_error'] + tuning_state['error_range']:
            
            # Start timer for oscillation
            if not tuning_state['start_timer']:
                tuning_state['start_timer'] = True
                tuning_state['start_time'] = tuning_state['timer']
                print("Start Timer for Oscillation")
        
        # Check for oscillation end
        elif tuning_state['max_error'] - tuning_state['error_range'] <= error <= tuning_state['max_error'] + tuning_state['error_range']:
            if tuning_state['start_timer']:
                # Oscillation cycle complete, (current time - start time = elapsed time)
                Tu = tuning_state['timer'] - tuning_state['start_time']
                print("Error Difference:", tuning_state['error_diff'])
                print(f"Oscillation Detected with Kp={tuning_state['kp']}, Tu={Tu}. Testing Complete. Please end the simulation.")
                tuning_state['oscillation_detected'] = True

    # Adjust Kp based on the error difference after 60 seconds if no oscillation detected
    if tuning_state['timer'] > tuning_state['process_time'] and not tuning_state['oscillation_detected']:
        adjust_kp_based_on_error_difference()

def adjust_kp_based_on_error_difference():
    global tuning_state
    error_diff = tuning_state['error_diff']
    print(f"Error Difference: {error_diff:.4f}")
    # Adjust Kp based on error difference (Subject to change based on tuning requirements)
    
    if error_diff > 1:
        tuning_state['kp'] += 0.1
    elif error_diff > 0.1:
        tuning_state['kp'] += 0.02
    elif error_diff > 0.01:
        tuning_state['kp'] += 0.001
    
    elif error_diff < -1:
        tuning_state['kp'] -= 0.1
    elif error_diff < -0.1:
        tuning_state['kp'] -= 0.01
    elif error_diff < -0.01:
        tuning_state['kp'] -= 0.001

    print(f"Adjusting Kp to {tuning_state['kp']:.4f} for next iteration")

    # Reset for next test
    reset_tuning_state()

def reset_tuning_state():
    global tuning_state
    tuning_state['min_error'] = float('inf')
    tuning_state['max_error'] = float('-inf')
    tuning_state['oscillation_detected'] = False
    tuning_state['start_time'] = None
    tuning_state['timer'] = 0
    tuning_state['start_timer'] = False
