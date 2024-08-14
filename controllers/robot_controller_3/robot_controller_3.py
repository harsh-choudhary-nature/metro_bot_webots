"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Supervisor, Keyboard
# import keyboard
import numpy as np
import sys
import time
import pickle


np.random.seed(42)

def sensor_value_to_index(sensor_value):
    value = 0
    if sensor_value<500:
        value = 400
    elif sensor_value>900:
        # value = 900
        value = 600
    else:
        # value = 500 + (int((sensor_value-500)/100))*100
        value = 500
    return (value-400)/100

def action_to_index(action):
    return int(action=="move")           # 0 to stop, 1 to move

def stopped_for_to_index(time):
    return int(time/32)

def save_Q_Table(table:dict):
    with open("q_table.pkl", "wb") as file:
        pickle.dump(table, file)

def load_Q_table():
    table = dict()
    try:
        with open("q_table.pkl", "rb") as file:
            table = pickle.load(file)
    except FileNotFoundError:
        pass
    return table

class State:
    def __init__(self):
        self.sensor_left = None
        self.sensor_right = None
        self.sensor_front = None
        self.station_arrived = None
        self.moved_after_stop = None
        self.prev_action = None
        self.stopped_for = None
        # # debugging
        # self.sensor_left_orig = None

class Environment:
    def __init__(self,robot):
        self.robot = robot
        self.us = []
        self.timestep = int(self.robot.getBasicTimeStep())
        self.usNames = ['ultrasonic right', 'ultrasonic front', 'ultrasonic left']
        for i in range(3):
            self.us.append(robot.getDevice(self.usNames[i]))
            self.us[i].enable(timestep)
        
    def get_initial_state(self):
        sensor_values = [self.us[i].getValue() for i in range(len(self.us))]
        state = State()
        state.sensor_left = sensor_value_to_index(sensor_values[2])
        state.sensor_right = sensor_value_to_index(sensor_values[0])
        state.sensor_front = sensor_value_to_index(sensor_values[1])
        state.station_arrived = 0
        state.moved_after_stop = 0
        state.prev_action = action_to_index("move")
        state.stopped_for = 0
        # # debugging
        # self.sensor_left_orig = sensor_values[2]
        return state

    def handle_sensor_left(self,state:State,action:str,nextState:State)->float:
        val = 0
        nextState.sensor_left = sensor_value_to_index(self.us[2].getValue())
        # # debugging
        # nextState.sensor_left_orig = self.us[2].getValue()
        return val

    def handle_sensor_right(self,state:State,action:str,nextState:State)->float:
        val = 0
        nextState.sensor_right = sensor_value_to_index(self.us[0].getValue())
        return val

    def handle_sensor_front(self,state:State,action:str,nextState:State)->float:
        val = 0
        nextState.sensor_front = sensor_value_to_index(self.us[1].getValue())
        if action=="move":
            if state.sensor_front < sensor_value_to_index(1000):
                val = state.sensor_front - sensor_value_to_index(1000)
        return val

    def handle_station_arrived(self,state:State,action:str,nextState:State)->float:
        val = 0
        if action == "stop":
            if state.station_arrived == 1:
                if min(state.sensor_left,state.sensor_right)<sensor_value_to_index(1000) and min(nextState.sensor_left,nextState.sensor_right)>=sensor_value_to_index(1000):
                    nextState.station_arrived = 0
                    if state.moved_after_stop == 1:
                        val = -1
                    else:
                        if stopped_for_to_index(state.stopped_for) == 5:
                            val = -1
                        else:
                            val = 1
                else:
                    nextState.station_arrived = 1
                    if state.moved_after_stop == 1:
                        val = -1
                    else:
                        if stopped_for_to_index(state.stopped_for) == 5:
                            val = -1
                        else:
                            val = 1
            else:
                if min(state.sensor_left,state.sensor_right)>=sensor_value_to_index(1000) and min(nextState.sensor_left,nextState.sensor_right)<sensor_value_to_index(1000):
                    nextState.station_arrived = 1
                    val = -1
                else:
                    nextState.station_arrived = 0
                    val = -1
        else:
            if state.station_arrived == 1:
                if min(state.sensor_left,state.sensor_right)<sensor_value_to_index(1000) and min(nextState.sensor_left,nextState.sensor_right)>=sensor_value_to_index(1000):
                    nextState.station_arrived = 0
                    if stopped_for_to_index(state.stopped_for)>0:
                        val = 1
                    else:
                        val = -10
                else:
                    nextState.station_arrived = 1 
                    val = 1 
            else:
                if min(state.sensor_left,state.sensor_right)>=sensor_value_to_index(1000) and min(nextState.sensor_left,nextState.sensor_right)<sensor_value_to_index(1000):
                    nextState.station_arrived = 1
                    val = 1
                else:
                    nextState.station_arrived = 0
                    val = 1
        # print(f"action {action} min(state.sensor_left,state.sensor_right) {min(state.sensor_left,state.sensor_right)} min(nextState.sensor_left,nextState.sensor_right) {min(nextState.sensor_left,nextState.sensor_right)} state.station_arrived {state.station_arrived} nextState.station_arrived {nextState.station_arrived} sensor_value_to_index(1000) {sensor_value_to_index(1000)}")
        return val
    
    def handle_moved_after_stop(self,state:State,action:str,nextState:State)->float:
        val = 0
        if action == "stop":
            if nextState.station_arrived == 1:
                nextState.moved_after_stop = state.moved_after_stop
            else:
                nextState.moved_after_stop = 0
        else:
            if nextState.station_arrived == 0:
                nextState.moved_after_stop = 0
            else:
                if state.moved_after_stop == 1:
                    nextState.moved_after_stop = 1
                else:
                    if state.prev_action == action_to_index("stop"):
                        nextState.moved_after_stop = 1
                    else:
                        nextState.moved_after_stop = 0
        return val

    def handle_prev_action(self,state:State,action:str,nextState:State)->float:
        val = 0
        nextState.prev_action = action_to_index(action)
        return val
    
    def handle_stopped_for(self,state:State,action:str,nextState:State)->float:
        val = 0
        if action == "stop":
            if nextState.station_arrived == 1:
                if state.moved_after_stop == 1:
                    val = -1
                    nextState.stopped_for = state.stopped_for
                else:
                    nextState.stopped_for = min(1+state.stopped_for,5*self.timestep)
                    if stopped_for_to_index(state.stopped_for) == 5:
                        val = -1
                    else:
                        val = 10
            else:
                nextState.stopped_for = 0
                val = -1
        else:
            if nextState.station_arrived == 1:
                nextState.stopped_for = state.stopped_for
                if stopped_for_to_index(state.stopped_for) < 5 and state.moved_after_stop == 0:
                    val = -10
                else:
                    val = 1 
            else:
                val = 1
                nextState.stopped_for = 0
        return val

    def get_reward_next_state(self,state:State,action:str):
        # sensor_values = [self.us[i].getValue() for i in range(len(self.us))]
        # print(sensor_values)
        state_tuple = (state.sensor_left,state.sensor_right,state.sensor_front,state.station_arrived,state.moved_after_stop,state.prev_action,stopped_for_to_index(state.stopped_for))

        if state.station_arrived == 1 and action == "stop" and state.moved_after_stop == 0:
            print("stopped for",stopped_for_to_index(state.stopped_for))
        # if state.station_arrived == 1 and action == "stop" and state.moved_after_stop == 1:
        #     print("restopping at same station",state_tuple)
        # if state.sensor_right<2:
        #     print("station on right")

        nextState = State()
        val = 0
        val += self.handle_sensor_left(state,action,nextState)
        val += self.handle_sensor_right(state,action,nextState)
        val += self.handle_sensor_front(state,action,nextState)
        val += self.handle_station_arrived(state,action,nextState)
        val += self.handle_moved_after_stop(state,action,nextState)
        val += self.handle_prev_action(state,action,nextState)
        val += self.handle_stopped_for(state,action,nextState)
        return val, nextState

class Agent:
    def __init__(self,robot,MAX_SPEED):
        self.robot = robot
        self.MAX_SPEED = MAX_SPEED
        self.timestep = int(self.robot.getBasicTimeStep())

        # enable devices
        # get motors
        self.left_motor = robot.getDevice('rotational motor left')
        self.right_motor = robot.getDevice('rotational motor right')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # get IR sensors
        self.gs = []
        self.gsNames = ['ir right', 'ir left']
        for i in range(2):
            self.gs.append(robot.getDevice(self.gsNames[i]))
            self.gs[i].enable(timestep)

        # states
        self.states = ['forward', 'turn_right', 'turn_left']
        self.current_state = self.states[0]

        # counter: used to maintain an active state for a number of cycles
        self.counter = 0
        self.COUNTER_MAX = 5
        self.STOPPED_MAX = int(self.robot.getBasicTimeStep())*5
        
        # related to Q learning
        self.actions = ["stop", "move"]
        self.QATable = load_Q_table()
        self.policy = dict()
        if len(self.QATable.keys())>0:
            for key in self.QATable:
                self.policy[key ] = "stop" if self.QATable[key]["stop"]>self.QATable[key]["move"] else "move"
        self.discount = 0.95
        self.alpha = 0.2
        self.epsilon = 0
        self.action_timer = 0
        self.cur_action = "move"

    def handle_new_state(self,state):
        assert state not in self.QATable
        self.QATable[state] = dict()
        for action in self.get_actions(state):
            self.QATable[state][action] = 0
        self.policy[state] = np.random.choice(self.get_actions(state))

    def get_actions(self,state):
        return self.actions

    def register_initial_state(self,state:tuple):
        self.QAState = state
        if state not in self.QATable:
            self.handle_new_state(state)

    def act(self, leftSpeed, rightSpeed):
        # epsilon greedy
        self.action_timer += 1
        action = None
        if self.action_timer == self.timestep:
            self.action_timer = 0
            p = np.random.rand()
            if p<self.epsilon:
                # take random action
                action = np.random.choice(self.get_actions(self.QAState))
            else:
                # take action according to best policy till now
                action = self.policy[self.QAState]
            self.cur_action = action
        else:
            action = self.cur_action
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

        return leftSpeed,rightSpeed

    def update_QATablePolicy(self,state:tuple,action:str,reward:float,next_state:tuple):
        max_QA_next_state = 0
        if next_state not in self.QATable:
            self.handle_new_state(next_state)
        
        max_QA_next_state = max([self.QATable[next_state][action] for action in self.get_actions(next_state)])
        self.QATable[state][action] = (1-self.alpha)*self.QATable[state][action] + self.alpha*(reward + self.discount*max_QA_next_state)

        if self.QATable[state][action]>self.QATable[state][self.policy[state]]:
            self.policy[state] = action

    def update_state(self,state):
        self.QAState = state

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# set the maximum velocity
MAX_SPEED = 6.28*0.25

agent = Agent(robot,MAX_SPEED)
environment = Environment(robot)

# Enable keyboard input
keyboard = Keyboard()
keyboard.enable(timestep)

robot.step(timestep)        # to start the simulation with some timesteps passed

initialState = environment.get_initial_state()
agent.register_initial_state((initialState.sensor_left,initialState.sensor_right,initialState.sensor_front,initialState.station_arrived,initialState.moved_after_stop,initialState.prev_action,stopped_for_to_index(initialState.stopped_for)))
curState = initialState

while robot.step(timestep) != -1:
    leftSpeed, rightSpeed = agent.line_follow()
    action = agent.act(leftSpeed,rightSpeed)
    reward, nextState = environment.get_reward_next_state(curState,action)
    next_state = (nextState.sensor_left,nextState.sensor_right,nextState.sensor_front,nextState.station_arrived,nextState.moved_after_stop,nextState.prev_action,stopped_for_to_index(nextState.stopped_for))
    agent.update_QATablePolicy(agent.QAState,action,reward,next_state)
    agent.update_state(next_state)
    curState = nextState

    # Check if the 'S' key is pressed to exit
    key = keyboard.getKey()
    if key == ord('S'):  # Press 'S' to exit
        print("S key detected, saving...")
        save_Q_Table(agent.QATable)
        break

print("Exiting...")