"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
import math

def move_forward(left_motor, right_motor,max_velocity):
    left_motor.setVelocity(max_velocity)
    right_motor.setVelocity(max_velocity)
    
def turn_left(left_motor, right_motor,max_velocity):
    left_motor.setVelocity(-max_velocity)
    right_motor.setVelocity(max_velocity)
    
def turn_right(left_motor, right_motor,max_velocity):
    left_motor.setVelocity(max_velocity)
    right_motor.setVelocity(-max_velocity)    

def stop(left_motor, right_motor,max_velocity):
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# set the maximum velocity
max_velocity = 6.28

# enable devices
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# get IR sensors
left_ir = robot.getDevice('ir left')
right_ir = robot.getDevice('ir right')

# enable IR sensors
left_ir.enable(timestep)
right_ir.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # read IR sensor values
    left_ir_value = left_ir.getValue()
    right_ir_value = right_ir.getValue()
    
    # print IR sensor values
    print(f"e-puck: Left IR: {left_ir_value}, Right IR: {right_ir_value}")

    move_forward(left_motor,right_motor,max_velocity)

