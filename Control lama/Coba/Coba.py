import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Input
ir_left = ctrl.Antecedent(np.arange(0, 101, 1), 'ir_left')
ir_right = ctrl.Antecedent(np.arange(0, 101, 1), 'ir_right')
gs_left = ctrl.Antecedent(np.arange(0, 1001, 1), 'gs_left')
gs_center = ctrl.Antecedent(np.arange(0, 1001, 1), 'gs_center')
gs_right = ctrl.Antecedent(np.arange(0, 1001, 1), 'gs_right')

# Output
left_motor_speed = ctrl.Consequent(np.arange(-100, 101, 1), 'left_motor_speed')
right_motor_speed = ctrl.Consequent(np.arange(-100, 101, 1), 'right_motor_speed')

# Membership functions
ir_left['low'] = fuzz.trimf(ir_left.universe, [0, 0, 50])
ir_left['medium'] = fuzz.trimf(ir_left.universe, [0, 50, 100])
ir_left['high'] = fuzz.trimf(ir_left.universe, [50, 100, 100])

ir_right['low'] = fuzz.trimf(ir_right.universe, [0, 0, 50])
ir_right['medium'] = fuzz.trimf(ir_right.universe, [0, 50, 100])
ir_right['high'] = fuzz.trimf(ir_right.universe, [50, 100, 100])

gs_left['black'] = fuzz.trimf(gs_left.universe, [0, 0, 500])
gs_left['white'] = fuzz.trimf(gs_left.universe, [400, 1000, 1000])

gs_center['black'] = fuzz.trimf(gs_center.universe, [0, 0, 500])
gs_center['white'] = fuzz.trimf(gs_center.universe, [400, 1000, 1000])

gs_right['black'] = fuzz.trimf(gs_right.universe, [0, 0, 500])
gs_right['white'] = fuzz.trimf(gs_right.universe, [400, 1000, 1000])

left_motor_speed['reverse'] = fuzz.trimf(left_motor_speed.universe, [-10, -10, -5])
left_motor_speed['stop'] = fuzz.trimf(left_motor_speed.universe, [-6, 0, 6])
left_motor_speed['forward'] = fuzz.trimf(left_motor_speed.universe, [5, 10, 10])

right_motor_speed['reverse'] = fuzz.trimf(right_motor_speed.universe, [-10, -10, -5])
right_motor_speed['stop'] = fuzz.trimf(right_motor_speed.universe, [-6, 0, 6])
right_motor_speed['forward'] = fuzz.trimf(right_motor_speed.universe, [5, 10, 10])

# Rules
rule1 = ctrl.Rule(ir_left['low'] & ir_right['low'] & gs_center['black'], 
                  (left_motor_speed['stop'], right_motor_speed['stop']))
rule2 = ctrl.Rule(ir_left['low'] & ir_right['medium'] & gs_center['black'], 
                  (left_motor_speed['forward'], right_motor_speed['stop']))
rule3 = ctrl.Rule(ir_left['medium'] & ir_right['low'] & gs_center['black'], 
                  (left_motor_speed['stop'], right_motor_speed['forward']))
rule4 = ctrl.Rule(ir_left['medium'] & ir_right['medium'] & gs_center['black'], 
                  (left_motor_speed['forward'], right_motor_speed['forward']))
# Define other rules based on the C code's logic

# Control System
motor_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
motor_speeding = ctrl.ControlSystemSimulation(motor_ctrl)

# Function to convert fuzzy output to motor speed
def fuzzy_to_motor_speed(output):
    if output >= 0:
        return output
    else:
        return -output  # Reverse direction for negative values

# Example Usage
ir_left_value = 70
ir_right_value = 30
gs_center_value = 300
motor_speeding.input['ir_left'] = ir_left_value
motor_speeding.input['ir_right'] = ir_right_value
motor_speeding.input['gs_center'] = gs_center_value
motor_speeding.compute()
left_speed = fuzzy_to_motor_speed(motor_speeding.output['left_motor_speed'])
right_speed = fuzzy_to_motor_speed(motor_speeding.output['right_motor_speed'])
print("Left Motor Speed:", left_speed)
print("Right Motor Speed:", right_speed)
