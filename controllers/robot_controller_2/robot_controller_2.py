"""robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Supervisor
import numpy as np
import sys
import time


np.random.seed(42)

class Station:
    def __init__(self, pos, length_y, timeout):
        self.pos = (pos[0],pos[1])
        self.pos1 = (pos[0],pos[1]-length_y/2)
        self.pos2 = (pos[0],pos[1]+length_y/2)
        self.TIMEOUT = timeout
        self.arrived = 0
        self.timer = 0
        self.stopped_for = 0
        self.entered_pos_index = 1
        self.moved_after_stopping = 0

    def reset(self):
        self.arrived = 0
        self.timer = 0
        self.stopped_for = 0
        self.entered_pos_index = 1
        self.moved_after_stopping = 0

class Environment:
    def __init__(self,robot,stations):
        self.robot = robot
        self.robot_node = robot.getSelf()
        self.stations = stations
        self.us = []
        self.usNames = ['ultrasonic right', 'ultrasonic front', 'ultrasonic left']
        for i in range(3):
            self.us.append(robot.getDevice(self.usNames[i]))
            self.us[i].enable(timestep)
        
        # [0.782431, 0.955251, 0.029863679999999997]
        # [0.782440516996644, 0.9552865211568394, 0.012192729170731165], so norm is 0.017
        self.TIMEOUT = int(self.robot.getBasicTimeStep())*20            # 20 s assumed distance between 2 stations
        self.WAITTIME = int(self.robot.getBasicTimeStep())*10           # 10 s is stopping time of robot
        self.station_positions = [stations[i].getPosition() for i in range(len(self.stations))]
        # print(self.station_positions)
        self.stations = []
        shape_node = robot.getFromDef("station_shape_1")
        assert shape_node is not None
        geometry_field = shape_node.getField("geometry")
        assert geometry_field is not None
        geometry_node = geometry_field.getSFNode()
        assert geometry_node is not None
        geometry_type = geometry_node.getTypeName()
        assert geometry_type=="Box"
        size_field = geometry_node.getField("size")
        assert size_field is not None
        size = size_field.getSFVec3f()
        length = size[1]
        for i in range(len(stations)):
            self.stations.append(Station(self.station_positions[i],length,self.TIMEOUT))
            print(self.stations[-1].pos1,self.stations[-1].pos2)
        
    def get_initial_state(self):
        sensor_values = [self.us[i].getValue() for i in range(len(self.us))]
        floored_sensor_values = []
        for value in sensor_values:
            if value<500:
                floored_sensor_values.append(400)
            elif value>900:
                floored_sensor_values.append(900)
            else:
                floored_sensor_values.append(500 + (int((value-500)/100))*100)
        return tuple(floored_sensor_values)

    def get_closest_station(self,robot_position):
        closest_station_index = -1
        closest_station_distance = 500
        for i in range(len(self.station_positions)):
            if self.stations[i].timer == self.TIMEOUT:
                self.stations[i].reset()
            if self.stations[i].arrived==1 or self.stations[i].arrived==2:
                # either arrived or left
                self.stations[i].timer += 1
            cur_dist = np.linalg.norm(np.array(robot_position)-np.array(self.station_positions[i]))            
            if closest_station_index==-1:
                closest_station_index = i
                closest_station_distance = cur_dist
            elif closest_station_distance>cur_dist:
                closest_station_index = i
                closest_station_distance = cur_dist

        return closest_station_index, closest_station_distance

    def get_reward_next_state(self,state,robot_position,action):
        next_state_sensor_values = [self.us[i].getValue() for i in range(len(self.us))]
        next_state_floored_sensor_values = []
        for value in next_state_sensor_values:
            if value<500:
                next_state_floored_sensor_values.append(400)
            elif value>900:
                next_state_floored_sensor_values.append(900)
            else:
                # print(value)
                next_state_floored_sensor_values.append(500 + (int((value-500)/100))*100)
        
        val = 0
        closest_station_index, closest_station_distance = self.get_closest_station(robot_position)
        # print(f"action {action} closest_station_distance {closest_station_distance}")

        robot_position = (robot_position[0],robot_position[1])
        current_position = self.robot_node.getPosition()
        current_position = (current_position[0],current_position[1])
        if action=="stop":
            if self.stations[closest_station_index].arrived==1 and self.stations[closest_station_index].stopped_for<self.WAITTIME and self.stations[closest_station_index].moved_after_stopping==0:
                val = 10/closest_station_distance
                self.stations[closest_station_index].stopped_for += 1
                print("stopped correct at station")
            elif self.stations[closest_station_index].arrived==1 and self.stations[closest_station_index].stopped_for>=self.WAITTIME and self.stations[closest_station_index].moved_after_stopping==0:
                val = -1
                print("stopped overtime at station")
            elif self.stations[closest_station_index].arrived==1 and self.stations[closest_station_index].moved_after_stopping==1:
                val = -1
                print("restopped at station")
            else:
                assert self.stations[closest_station_index].arrived!=1
                val = -1
                print("stoppped when not the station")
        else:
            if self.stations[closest_station_index].arrived==0 and robot_position[1]>min(self.stations[closest_station_index].pos1[1],self.stations[closest_station_index].pos2[1]) and robot_position[1]<max(self.stations[closest_station_index].pos1[1],self.stations[closest_station_index].pos2[1]):
                self.stations[closest_station_index].arrived = 1
                val = 1
                dist1 = np.linalg.norm(np.array(self.stations[closest_station_index].pos1)-np.array(robot_position))
                dist2 = np.linalg.norm(np.array(self.stations[closest_station_index].pos2)-np.array(robot_position))
                self.stations[closest_station_index].entered_pos_index = 1 if dist1<dist2 else 2
                print("move arrived at the station, pos_index = ",self.stations[closest_station_index].entered_pos_index)
            elif self.stations[closest_station_index].arrived==1 and ((self.stations[closest_station_index].entered_pos_index==1 and ((robot_position[1]-self.stations[closest_station_index].pos2[1])*(current_position[1]-self.stations[closest_station_index].pos2[1])<=0)) or (self.stations[closest_station_index].entered_pos_index==2 and ((robot_position[1]-self.stations[closest_station_index].pos1[1])*(current_position[1]-self.stations[closest_station_index].pos1[1])<=0))):
                self.stations[closest_station_index].arrived = 2
                if self.stations[closest_station_index].stopped_for>0:
                    self.stations[closest_station_index].moved_after_stopping = 1
                    val = -50
                    print("move station over not stopped at the station")
                else:
                    val = 1
                    print("move station over and stopped at the station")
            elif self.stations[closest_station_index].arrived==1 and self.stations[closest_station_index].stopped_for>0:
                self.stations[closest_station_index].moved_after_stopping = 1
                if self.stations[closest_station_index].stopped_for > self.WAITTIME:
                    val = 2
                    print("moved overtime at station")
                else:
                    print("moved early from station")
                    val = -2
            else:
                if self.stations[closest_station_index].arrived==1:
                    print("moving at station")
                else:
                    print("moving on track")
                val = 1

        return val,tuple(next_state_floored_sensor_values)

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

        # states
        self.states = ['forward', 'turn_right', 'turn_left']
        self.current_state = self.states[0]

        # counter: used to maintain an active state for a number of cycles
        self.counter = 0
        self.COUNTER_MAX = 5
        self.STOPPED_MAX = int(self.robot.getBasicTimeStep())*10
        self.last_act_time = 0

        # related to Q learning
        self.actions = ["stop", "move"]
        self.QATable = dict()
        self.exploration_table = dict()
        self.policy = dict()
        self.discount = 0.8
        self.alpha = 0.1

    def handle_new_state(self,state):
        assert state not in self.QATable
        self.QATable[state] = dict()
        for action in self.get_actions(state):
            self.QATable[state][action] = 0
        self.policy[state] = np.random.choice(self.get_actions(state))
        self.exploration_table[state] = 2

    def get_actions(self,state):
        return self.actions

    def register_initial_state(self,state:tuple):
        self.QAState = state
        self.handle_new_state(state)

    def act(self, leftSpeed, rightSpeed):
        # epsilon greedy
        p = np.random.rand()
        action = None
        if p<1/self.exploration_table[self.QAState]:
            # take random action
            action = np.random.choice(self.get_actions(self.QAState))
        else:
            # take action according to best policy till now
            action = self.policy[self.QAState]
        if action == "move":
            self.last_act_time = 0
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

    def update_QATablePolicy(self,state,action,reward,next_state):
        max_QA_next_state = 0
        if next_state not in self.QATable:
            self.handle_new_state(next_state)
        
        max_QA_next_state = max([self.QATable[next_state][action] for action in self.get_actions(next_state)])
        self.exploration_table[next_state] += 1
        
        self.QATable[state][action] = (1-self.alpha)*self.QATable[state][action] + self.alpha*(reward + self.discount*max_QA_next_state)

        if self.QATable[state][action]>self.QATable[state][self.policy[state]]:
            self.policy[state] = action

    def update_state(self,state):
        self.QAState = state

# create the Robot instance.
robot = Supervisor()
stations = [robot.getFromDef("station_1"),robot.getFromDef("station_2"),robot.getFromDef("station_3"),robot.getFromDef("station_4")]

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# set the maximum velocity
MAX_SPEED = 6.28*0.25

agent = Agent(robot,MAX_SPEED)
environment = Environment(robot,stations)

robot.step(timestep)        # to start the simulation with some timesteps passed

agent.register_initial_state(environment.get_initial_state())

while robot.step(timestep) != -1:
    leftSpeed, rightSpeed = agent.line_follow()
    position = environment.robot_node.getPosition()
    action = agent.act(leftSpeed,rightSpeed)
    reward, next_state = environment.get_reward_next_state(agent.QAState,position,action)
    agent.update_QATablePolicy(agent.QAState,action,reward,next_state)
    agent.update_state(next_state)