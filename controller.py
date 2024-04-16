wind_active = False  # Select whether you want to activate wind or not
group_number = 32  # Enter your group number here
tune = False  # Set to True to tune the controller (Only works for y-axis controller)
#0.292 posy

# Global state for tuning
tuning_state = {
    'kp': 0.198,  # Starting Kp
    'min_error': float('inf'), # Min error to check for oscillation (bottom point of oscillation cycle)
    'max_error': float('-inf'), # Max error to check for oscillation (top point of oscillation cycle)
    'oscillation_detected': False, # Flag to check if oscillation detected
    'start_time': None, # Start timer flag for oscillation
    'timer': 0, # Timer to check for one cycle of oscillation
    'start_timer': False,
    'error_range': 0.01,  # Range within which we consider the error to be close to min or max
    'error_mag': 0.002,  # Error magnitude, when error difference is less than this, we check for oscillation
    'error_diff': 0.1,  # Error difference to check for oscillation, gets updated based on min and max error
    'process_time': 20  # Time to check for oscillation, if no oscillation detected, adjust Kp
}

# PID Controller class, easy to use and edit values directly as we have multiple PID controllers
class PIDController:
    # Initialize the PID controller
    def __init__(self, kp=0, ki=0, kd=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = None
        self.i_u = 0
    # Calculate the PID output
    def calculate(self, error, dt):
        
        # Set prev error for first iteration
        if self.prev_error is None:
            self.prev_error = error
        
        # Proportional term
        p_u = self.kp * error

        # Integral term
        self.i_u += self.ki * error * dt

        # Derivative term
        d_u = self.kd * (error - self.prev_error) / dt
        
        # Calculate PID output
        pid_u = p_u + self.i_u + d_u

        self.prev_error = error

        return pid_u
    
    # Edit values directly using these functions
    def set_kp(self, invar):
        self.kp = invar
        
    def set_ki(self, invar):
        self.ki = invar
        
    def set_kd(self, invar):
        self.kd = invar
        
# PID values for tuning       
#5,3,2 for pid_y also works well / 0.175, 0.112, 0.068
pid_y = PIDController(kp=0.175, ki=0.112, kd=0.068)
#0.2 // 0.12, 0.348, 0.0102 // 0.03
pid_x = PIDController(kp=1.2, ki=0.4, kd=1)
#0.75, 0.28, 0.095 / 0.5, 0.25, 0.09
pid_attitude = PIDController(kp=9, ki=4, kd=3)


# Function to normalize an angle to [-π, π] range
def normalize_angle(angle):
    pi = 3.141592653589793
    return (angle + pi) % (2 * pi) - pi


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
    
    # Offset temporary solution
    #if target_pos[0] < 4:
        #errorX -= 0.6

    # Normalize the current attitude and calculate the error
    current_normalized_attitude = normalize_angle(state[4])
    
    
    thrust = pid_y.calculate(errorY, dt)
    pitch = pid_x.calculate(errorX, dt)
    pid_output_attitude = pid_attitude.calculate(current_normalized_attitude, dt)  # Control drone's attitude
    
    #Just a test to see if we can cap the pitch#
    # if attitude is greater than 0.2 radians or less than -0.2 radians, reduce thrust and try to level out
    '''
    if(state[4] > 1):
        thrust *= 0.8  # reduce thrust by 20%
        pitch = 0.8
    elif(state[4] < -1):
        thrust *= 0.8  # reduce thrust by 20%
        pitch = -0.8
    '''
    
    # Thrust adjustments for lateral movement
    motor1_pwm = thrust - (pitch + pid_output_attitude)
    motor2_pwm = thrust + (pitch + pid_output_attitude)

    action = [motor1_pwm, motor2_pwm]
    
    # Check for oscillation if not already detected
    if not tuning_state['oscillation_detected'] and tune:
        pid_x.SetKp(tuning_state['kp']) # Set to tuning Kp
        check_for_oscillation(errorX, dt)
    
    return action
    
# Function to check if there is oscillation, if no oscillation detected, adjust the Kp based on error difference
def check_for_oscillation(error, dt):
    global tuning_state
    
    # Update min and max errors
    tuning_state['min_error'] = min(tuning_state['min_error'], error)
    tuning_state['max_error'] = max(tuning_state['max_error'], error)
    tuning_state['error_diff'] = tuning_state['max_error'] + tuning_state['min_error']
    
    # Increment timer
    tuning_state['timer'] += dt
    
    # Check for oscillation after 10 seconds have passed
    if tuning_state["timer"] > 10 and abs(tuning_state['error_diff']) <= tuning_state['error_mag']:
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
                tu = (tuning_state['timer'] - tuning_state['start_time']) * 2
                print("Error Difference:", tuning_state['error_diff'])
                print(f"Oscillation Detected with Kp={tuning_state['kp']}, Tu={tu}. Testing Complete. Please end the simulation.")
                tuning_state['oscillation_detected'] = True

    # Adjust Kp based on the error difference after 60 seconds if no oscillation detected
    if tuning_state['timer'] > tuning_state['process_time'] and not tuning_state['oscillation_detected']:
        adjust_kp_based_on_error_difference()

def adjust_kp_based_on_error_difference():
    global tuning_state
    error_diff = tuning_state['error_diff']
    print(f"Error Difference: {error_diff:.4f}")

    # Define scaling factors for Kp adjustment
    scale_factor = 0.02
    
    # Directly use error_diff to adjust Kp
    if abs(error_diff) > 0.001:  # Ensure there's a significant error before adjusting
        adjustment = abs(error_diff) * scale_factor
        if error_diff < 0:
            tuning_state['kp'] += adjustment
        elif error_diff > 0:
            tuning_state['kp'] -= adjustment

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
