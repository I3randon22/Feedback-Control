wind_active = False  # Select whether you want to activate wind or not
group_number = 32  # Enter your group number here
tune = False  # Set to True to tune the controller
#0.292 posy

# Global state for tuning
tuning_state = {
    'kp': 0.0008,  # Starting Kp
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

class PIDController:
    def __init__(self, Kp=0, Ki=0, Kd=0, max_limit=float('inf'), min_limit=float('-inf')):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_limit = max_limit
        self.min_limit = min_limit
        self.prev_error = None
        self.i_u = 0

    def calculate(self, error, dt):
        
        # Set prev error for first iteration
        if self.prev_error is None:
            self.prev_error = error
        
        # Proportional term
        p_u = self.Kp * error

        # Integral term
        self.i_u += self.Ki * error * dt
        self.i_u = max(min(self.i_u, self.max_limit), self.min_limit)  # Integral windup

        # Derivative term
        d_u = self.Kd * (error - self.prev_error) / dt

        pid_u = p_u + self.i_u + d_u

        self.prev_error = error

        return pid_u
    
    def SetKp(self, invar):
        self.Kp = invar
    
pid_y = PIDController(Kp=0.175, Ki=0.112, Kd=0.068, max_limit=1/0.112, min_limit=0)
pid_x = PIDController(Kp=0.01, Ki=0, Kd=0, max_limit=1, min_limit=0)
pid_attitude = PIDController(Kp=10, Ki=0, Kd=0, max_limit=1, min_limit=0)

# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor
    global tuning_state
    global pid_y
    global tune
    
    # Calculate the error (Change accorindly to your needs)
    errorY = state[1] - target_pos[1]
    errorX = state[0] - target_pos[0]
    errorAttitude = state[4] * 0.3  # Assuming index 4 is attitude 

    pid_output_y = pid_y.calculate(errorY, dt)
    pid_output_x = pid_x.calculate(errorX, dt)
    #pid_output_attitude = pid_attitude.calculate(errorAttitude, dt)  # Control drone's attitude

    # Thrust adjustments for lateral movement
    left_thrust = pid_output_y - pid_output_x - errorAttitude
    right_thrust = pid_output_y + pid_output_x + errorAttitude

    action = [left_thrust, right_thrust]
    
    
    
    # Check for oscillation if not already detected
    if not tuning_state['oscillation_detected'] and tune:
        pid_x.SetKp(tuning_state['kp']) # Set to tuning Kp
        check_for_oscillation(errorX, dt)
    
    return action
    

def check_for_oscillation(error, dt):
    global tuning_state
    
     # Update min and max errors
    tuning_state['min_error'] = min(tuning_state['min_error'], error)
    tuning_state['max_error'] = max(tuning_state['max_error'], error)
    tuning_state['error_diff'] = tuning_state['max_error'] + tuning_state['min_error']
    
    # Increment timer
    tuning_state['timer'] += dt
    
    # Check for oscillation after 10 seconds have passed
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
                Tu = (tuning_state['timer'] - tuning_state['start_time']) * 2
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
        
    # Define scaling factors for Kp adjustment
    scale_factor = 0.02

    # Directly use error_diff to adjust Kp
    if abs(error_diff) > 0.001:  # Ensure there's a significant error before adjusting
        adjustment = error_diff * (scale_factor if error_diff > 0 else -scale_factor)
        tuning_state['kp'] += adjustment

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
