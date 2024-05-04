wind_active = False  # Select whether you want to activate wind or not
group_number = 32  # Enter your group number here
tune = True  # Set to True to tune the controller (Only works for y-axis controller)
global previous_state  # Add this line at the top of your file
global previous_target_pos  # Add this line at the top of your file
#0.292 posy

previous_target_pos = None  # Add this line at the top of your file
# pi
import math
import numpy as np

# Global state for tuning
tuning_state = {
    'kp': 0.2,  # Starting Kp
    'min_error': float('inf'), # Min error to check for oscillation (bottom point of oscillation cycle)
    'max_error': float('-inf'), # Max error to check for oscillation (top point of oscillation cycle)
    'oscillation_detected': False, # Flag to check if oscillation detected
    'start_time': None, # Start timer flag for oscillation
    'timer': 0, # Timer to check for one cycle of oscillation
    'timer2' : 0,
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
        # I found theoretically first few seconds error is too large which produce too much negative impact
        # for I, we only need I work to make up the static error wich occur when drone close to target
        t = tuning_state['timer']
        if t > 0:
            self.i_u += self.ki * error * dt
        else:
            self.i_u += 0
        print("time:{:.1f}".format(t))


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
        

#PID without toggle wind
# if wind_active == False:
#     # PID values for tuning       
#     #5,3,2 for pid_y also works well / 0.175, 0.112, 0.068 // 5,0.33,2.5 // good 5 1.8 2 // 5 3.5 3
#     pid_y = PIDController(kp=5, ki=1.8, kd=2)
#     #0.2 // 0.12, 0.348, 0.0102 // 0.03 // recent:1.2 0.4 1 // 1.5 0.01 2 // 1.2 0.1 1.05
#     pid_x = PIDController(kp=1.2, ki=0, kd=1.05)
#     #0.75, 0.28, 0.095 / 0.5, 0.25, 0.09 // 9 4 3 
#     pid_attitude = PIDController(kp=10, ki=0.1, kd=25)

#PID without toggle wind
if wind_active == False:
    # PID values for tuning       
    #5,3,2 for pid_y also works well / 0.175, 0.112, 0.068 // 5,0.33,2.5 // good 5 1.8 2 // 5 3.5 3
    pid_y = PIDController(kp=5, ki=1.5, kd=3)
    #0.2 // 0.12, 0.348, 0.0102 // 0.03 // recent:1.2 0.4 1 // 1.5 0.01 2 // 1.2 0.1 1.05
    pid_x = PIDController(kp=1.5, ki=0, kd=2)
    #0.75, 0.28, 0.095 / 0.5, 0.25, 0.09 // 9 4 3 // 0.18 0.17 0.52
    pid_attitude = PIDController(kp=0.33, ki=0, kd=0.5)

#PID with toggled wind
else:
    # PID values for tuning       
    #5,3,2 for pid_y also works well / 0.175, 0.112, 0.068 // 5,0.33,2.5 // 5 1.8 2 // 5,2.5,2.5
    pid_y = PIDController(kp=5, ki=1.5, kd=3)
    #0.2 // 0.12, 0.348, 0.0102 // 0.03 // recent:1.2 0.4 1 // 1.5 0.01 2// 1.2 0 1.05
    pid_x = PIDController(kp=1.5, ki=0.1, kd=2)
    #0.75, 0.28, 0.095 / 0.5, 0.25, 0.09 // 9 4 3 // 15 0 5
    pid_attitude = PIDController(kp=0.8, ki=4, kd=0.5)
    
def model(state, action, dt):
    # Unpack the state and action
    position_x = state[0]
    velocity_x = state[2]
    attitude = state[4]
    
    throttle_left, throttle_right = action

    # Calculate the acceleration due to the throttle settings
    # sin(attitude) to get horizontal thrust
    acceleration_x = (throttle_left + throttle_right) * np.sin(attitude)

    # Calculate the change in orientation due to the difference in throttle settings
    delta_attitude = (throttle_left - throttle_right) * dt

    # Update the velocity and position
    next_velocity_x = velocity_x + acceleration_x * dt

    next_position_x = position_x + velocity_x * dt + 0.5 * acceleration_x * dt**2

    # Update the orientation
    next_attitude = normalize_angle(attitude + delta_attitude)

    # Pack the next state
    next_state = (next_position_x, next_velocity_x, next_attitude) 

    return next_state
    
# Implement the disturbance observer
def disturbance_observer(previous_state, current_state, action, dt):
    # Estimate the next state using the model
    estimated_next_state = model(previous_state, action, dt)
    
    current_state = (current_state[0], current_state[2], current_state[4])
    
    # Calculate the disturbance as the difference between the estimated next state and the actual next state
    disturbance = tuple(a - b for a, b in zip(current_state, estimated_next_state))
    
    # Calculate the norm of the disturbance
    disturbance_norm = np.linalg.norm(disturbance)
    
    return disturbance_norm


# Function to normalize an angle to [-π, π] range
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# use x error to calcalute target attitude
# def cal_XtoA (pidx):
#     pi = 3.141592653589793
#     alpha = -0.04
#     A_target = alpha * pidx
#     max_altitude = pi/12
#     min_altitude = -pi/12
#     if A_target < max_altitude and A_target > min_altitude:
#         return A_target
#     else:
#         return (A_target/abs(A_target)) * (pi/12)

# we use X pid calculation output as target attitude
def cal_XtoA (pidx):
    #max pitch attitued
    max_altitude = math.pi/12
    #limit max pitch angle 
    return -1.2 * max_altitude * math.tanh(pidx)



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
    global previous_state
    global previous_target_pos
    
    previous_state = state
    
    if previous_target_pos is None:
        previous_target_pos = target_pos
    
    if target_pos != previous_target_pos:
        tuning_state['timer'] = 0
    
    # Calculate the error (Change accorindly to your needs)
    errorY = (state[1] - target_pos[1])
    errorX = (state[0] - target_pos[0])
    # Offset temporary solution
    #if target_pos[0] < 4:
        #errorX -= 0.6

    # Normalize the current attitude and calculate the error
    current_normalized_attitude = normalize_angle(state[4])
    
    
    thrust = pid_y.calculate(errorY, dt)
    pitch = pid_x.calculate(errorX, dt)
    target_attitude = cal_XtoA(pitch)

    error_attitude = current_normalized_attitude - target_attitude

    pid_output_attitude = pid_attitude.calculate(error_attitude, dt)  # Control drone's attitude
    #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # if errorX < 0.2 or errorX > -0.2:
    #     pid_output_attitude = 0.5 * pid_attitude.calculate(current_normalized_attitude, dt)
    # else:
    #     pid_output_attitude = pid_attitude.calculate(current_normalized_attitude, dt)
    
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
    #motor1_pwm = thrust - (pitch + pid_output_attitude)
    #motor2_pwm = thrust + (pitch + pid_output_attitude)

    # if(current_normalized_attitude > 0.17):
    #     motor1_pwm = 0
    #     motor2_pwm = 1
    
    # elif(current_normalized_attitude < -0.16):
    #     motor1_pwm = 1
    #     motor2_pwm = 0

    # else:
    # error = current position - target position
    #motor1_pwm = thrust - (pitch + pid_output_attitude)
    #motor2_pwm = thrust + (pitch + pid_output_attitude)
    
    # Only calculate the disturbance if previous_state is not None
    if previous_state is not None and wind_active:
        # Estimate the disturbance
        #strong 5
        disturbance = disturbance_observer(previous_state, state, (thrust, pid_output_attitude), dt) * 1.2
        print(disturbance,state[4])
        
        # Adjust the action based on the estimated disturbance
        pid_output_attitude += disturbance
    
    
    motor1_pwm = max(0,min(1,thrust - pid_output_attitude))
    motor2_pwm = max(0,min(1,thrust + pid_output_attitude))


    action = [motor1_pwm, motor2_pwm]
    
    
    # Update previous_state at the end of the function
    previous_state = state

    previous_target_pos = target_pos

    tuning_state['timer'] += dt
    t = tuning_state['timer']
    #print("errorX:{:.2f}".format(errorX)," errorY:{:.2f}".format(errorY)," left - right:{:.4f} ".format(motor1_pwm - motor2_pwm),"T-0.3:{:.2f}".format(motor1_pwm-0.3), " pidx:{:.2f}".format(pitch)," current_normalized_attitude:{:.4f}".format(current_normalized_attitude), " pida:{:.2f}".format(pid_output_attitude))
    print("time:{:.1f}".format(t),"errorX:{:.2f}".format(errorX),"errorY:{:.2f}".format(errorY)," target_attitude:{:.2f}".format(target_attitude)," c_n_attitude:{:.4f}".format(current_normalized_attitude), " error_attitude:{:.2f}".format(error_attitude)," pid_output_attitude:{:.6f}".format(pid_output_attitude), " thrust:{:.4f}".format(thrust))
    # Check for oscillation if not already detected
    if not tuning_state['oscillation_detected'] and tune:
        pid_y.set_kp(tuning_state['kp']) # Set to tuning Kp
        check_for_oscillation(errorY, dt)
    
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
    print("tu: ", tu)

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


#  we will send a random desired position to your quadcopter (within +/- 4 meters of your vehicle).
#  (x +/- 4, y) or (x, y+/-4)  ??
#  (x+/-4, y+/-4) ??
#  (x +/- 2, y) or (x, y+/-2)  ??