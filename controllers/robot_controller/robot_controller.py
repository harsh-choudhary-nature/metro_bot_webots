"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot
import numpy as np
import sys
import time

class Environment:
    def __init__(self,agent):
        self.agent = agent

    def get_initial_state(self):
        # print(f"self.agent.us[0].getValue(): {self.agent.us[0].getValue()}")
        sensor_values = [self.agent.us[i].getValue() for i in range(len(self.agent.us))]
        floored_sensor_values = []
        for value in sensor_values:
            if value<500:
                floored_sensor_values.append(450)
            elif value>900:
                floored_sensor_values.append(900)
            else:
                floored_sensor_values.append((value-500)/50)
        return tuple(floored_sensor_values)+(False,30)

    def get_reward_next_state(self,state,action):
        next_state_sensor_values = [self.agent.us[i].getValue() for i in range(len(self.agent.us))]
        next_state_floored_sensor_values = []
        for value in next_state_sensor_values:
            if value<500:
                next_state_floored_sensor_values.append(450)
            elif value>900:
                next_state_floored_sensor_values.append(900)
            else:
                next_state_floored_sensor_values.append((value-500)/50)
        
        val = 0
        if action=="stop":
            val -= 1
            if min(state[:3])<900:
                val += 1000/min(state[:3])
                # print("Correct stopped")
                # time.sleep(1)
            if state[3] and state[4]==self.agent.STOPPED_MAX:
                val -= 1        # penalise for waiting too long
                print("False stop")
                # time.sleep(1)
            # penalise if restopping at the same station again, so modify agent to include these functionality of 30s wait at one station and no restopping via actions

        else:
            val += 1
            # penalise for missing the station
            if not state[3] and min((state[0],state[2]))<900 and min((next_state_floored_sensor_values[0],next_state_floored_sensor_values[2]))==900:
                val -= 3
                print("Missed the station")
                # time.sleep(1)
            # penalise if moving from station too early
            if state[3] and min((state[0],state[2]))<900 and state[4]<self.agent.STOPPED_MAX:
                val -= 10*(self.agent.STOPPED_MAX - state[4])/self.agent.STOPPED_MAX
                print("Early moving")
                # time.sleep(1)

        next_state = next_state_floored_sensor_values
        if action=="stop":
            next_state.append(True)
            if not state[3]:
                next_state.append(0)
            else:
                next_state.append(min(state[4]+1,self.agent.STOPPED_MAX))
        else:
            next_state.append(False)
            next_state.append(state[4])
            
        return val,tuple(next_state)

class Agent:
    def __init__(self,robot,MAX_SPEED):
        self.robot = robot
        self.MAX_SPEED = MAX_SPEED
        self.cur_speed = 0

        # enable devices
        # get motors
        self.left_motor = robot.getDevice('rotational motor left')
        self.right_motor = robot.getDevice('rotational motor right')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(self.cur_speed)
        self.right_motor.setVelocity(self.cur_speed)

        # get IR sensors
        self.gs = []
        self.gsNames = ['ir right', 'ir left']
        for i in range(2):
            self.gs.append(robot.getDevice(self.gsNames[i]))
            self.gs[i].enable(timestep)
            
        self.us = []
        self.usNames = ['ultrasonic right', 'ultrasonic front', 'ultrasonic left']
        for i in range(3):
            self.us.append(robot.getDevice(self.usNames[i]))
            self.us[i].enable(timestep)

        # states
        self.states = ['forward', 'turn_right', 'turn_left']
        self.current_state = self.states[0]

        # counter: used to maintain an active state for a number of cycles
        self.counter = 0
        self.COUNTER_MAX = 5
        self.STOPPED_MAX = 960

        # related to Q learning
        self.actions = ["stop", "move"]
        self.QATable = dict()
        self.policy = dict()
        self.discount = 0.8
        self.epsilon = 0.4
        self.alpha = 0.1

    def register_initial_state(self,state:tuple):
        self.QAState = state
        self.QATable[self.QAState] = dict()
        for action in self.actions:
            self.QATable[self.QAState][action] = 0          # initialising with empty dictionary
        self.policy[self.QAState] = np.random.choice(self.actions)

    def act(self, leftSpeed, rightSpeed):
        # epsilon greedy
        p = np.random.rand()
        action = None
        if p<self.epsilon:
            # take random action
            action = np.random.choice(self.actions)
        else:
            # take action according to best policy till now
            action = self.policy[self.QAState]
        if action == "move":
            self.left_motor.setVelocity(leftSpeed)
            self.right_motor.setVelocity(rightSpeed)
        else:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
        return action

    def line_follow(self):
        gsValues = []
        for i in range(2):
            gsValues.append(self.gs[i].getValue())

        # Process sensor data
        line_right = gsValues[0] > 600
        line_left = gsValues[1] > 600
        # print(f"gsValues {gsValues}")
        # Implement the line-following state machine
        if self.current_state == 'forward':
            # Action for the current state: update speed variables
            leftSpeed = self.MAX_SPEED
            rightSpeed = self.MAX_SPEED

            # check if it is necessary to update current_state
            if line_right and not line_left:
                self.current_state = 'turn_right'
                self.counter = 0
            elif line_left and not line_right:
                self.current_state = 'turn_left'
                self.counter = 0
                
        if self.current_state == 'turn_right':
            # Action for the current state: update speed variables
            leftSpeed = 0.8 * MAX_SPEED
            rightSpeed = 0.4 * MAX_SPEED

            # check if it is necessary to update current_state
            if self.counter == self.COUNTER_MAX:
                self.current_state = 'forward'

        if self.current_state == 'turn_left':
            # Action for the current state: update speed variables
            leftSpeed = 0.4 * MAX_SPEED
            rightSpeed = 0.8 * MAX_SPEED

            # check if it is necessary to update current_state
            if self.counter == self.COUNTER_MAX:
                self.current_state = 'forward'        

        # increment counter
        self.counter += 1
        
        #print('Counter: '+ str(counter), gsValues[0], gsValues[1], gsValues[2])
        # print('Counter: '+ str(counter) + '. Current state: ' + current_state)

        # Set motor speeds with the values defined by the state-machine
        return leftSpeed,rightSpeed

    def update_QATablePolicy(self,state,action,reward,next_state):
        max_QA_next_state = 0
        if next_state in self.QATable:
            max_QA_next_state = max([self.QATable[next_state][action] for action in self.actions])
        else:
            self.QATable[next_state] = dict()
            self.policy[next_state] = np.random.choice(self.actions)
            for action in self.actions:
                self.QATable[next_state][action] = 0

        self.QATable[state][action] = (1-self.alpha)*self.QATable[state][action] + self.alpha*(reward + self.discount*max_QA_next_state)

        if self.QATable[state][action]>self.QATable[state][self.policy[state]]:
            self.policy[state] = action

    def update_state(self,state):
        self.QAState = state

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print("timestep = ",timestep)

# set the maximum velocity
MAX_SPEED = 6.28*0.25

agent = Agent(robot,MAX_SPEED)
environment = Environment(agent)

robot.step(timestep)        # to start the simulation with some timesteps passed

agent.register_initial_state(environment.get_initial_state())

# sys.exit(0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # print(f"agent.QAState = {agent.QAState}")
    # print(f"agent.QATable: {agent.QATable}")

    # print(f"len(agent.QATable.keys()): {len(agent.QATable.keys())}")
    leftSpeed, rightSpeed = agent.line_follow()
    action = agent.act(leftSpeed,rightSpeed)
    reward, next_state = environment.get_reward_next_state(agent.QAState,action)
    # print("action = ",action," reward = ",reward)
    # print(agent.policy)
    agent.update_QATablePolicy(agent.QAState,action,reward,next_state)
    agent.update_state(next_state)
    # sys.exit()