try:
    import sim as vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import math
import numpy as np
import random
import time


class Environment(object):
    def __init__(self):

        self.clientID = -1
        self.maxDetectionDist = 0.1
        self.noDetectionDist = 0.3
        self.prox_sensors = []
        self.number_of_proximity_sensors = 16
        self.start_simulation()
        self.initialize_program()
        self.time_without_collision = 0
        
    def start_simulation(self):
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        
    def stop_simulation(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
    
    def terminate_simulation(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        vrep.simxFinish(self.clientID)

    def initialize_program(self):
    
        print('Program started')

        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

        # Initialising object handles-----------------------------------------------------------------------------------------------------------
        _, self.left_motor = vrep.simxGetObjectHandle(self.clientID, './leftMotor', vrep.simx_opmode_blocking)
        _, self.right_motor = vrep.simxGetObjectHandle(self.clientID, './rightMotor', vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor, 0, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor, 0, vrep.simx_opmode_blocking)

        _, self.Robot = vrep.simxGetObjectHandle(self.clientID, './PioneerP3DX', vrep.simx_opmode_blocking)
        # _, self.Target = vrep.simxGetObjectHandle(self.clientID, 'Target', vrep.simx_opmode_oneshot_wait)

        _, self.caster_wheel_joint_1 = vrep.simxGetObjectHandle(self.clientID, './caster_freeJoint1',vrep.simx_opmode_blocking)
        _, self.caster_wheel_joint_2 = vrep.simxGetObjectHandle(self.clientID, './caster_freeJoint2',vrep.simx_opmode_blocking)

        _, self.f_point = vrep.simxGetObjectHandle(self.clientID, './connection[4]', vrep.simx_opmode_blocking)
        _, self.r_point = vrep.simxGetObjectHandle(self.clientID, './connection[9]', vrep.simx_opmode_blocking)

        for i in range(16):
            self.prox_sensors = self.prox_sensors + [vrep.simxGetObjectHandle(self.clientID, './ultrasonicSensor[' + str(i) + ']' ,vrep.simx_opmode_blocking)[1]]
            
        for i in range(self.number_of_proximity_sensors):
            vrep.simxReadProximitySensor(self.clientID, self.prox_sensors[i], vrep.simx_opmode_streaming)
        #vrep.simxGetObjectPosition(self.clientID, self.Target, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.Robot, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectVelocity(self.clientID, self.Robot, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.right_motor, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.left_motor, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.caster_wheel_joint_1, -1, vrep.simx_opmode_streaming)
        vrep.simxGetObjectPosition(self.clientID, self.caster_wheel_joint_2, -1, vrep.simx_opmode_streaming)

        #vrep.simxGetObjectPosition(self.clientID, self.Target, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.Robot, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectVelocity(self.clientID, self.Robot, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.right_motor, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.left_motor, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID,self. caster_wheel_joint_1, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.caster_wheel_joint_2, -1, vrep.simx_opmode_blocking)

        #vrep.simxGetObjectPosition(self.clientID, self.Target, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.Robot, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectVelocity(self.clientID, self.Robot, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.right_motor, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.left_motor, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.caster_wheel_joint_1, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.caster_wheel_joint_2, -1, vrep.simx_opmode_blocking)

        vrep.simxGetObjectPosition(self.clientID, self.f_point, -1, vrep.simx_opmode_blocking)
        vrep.simxGetObjectPosition(self.clientID, self.r_point, -1, vrep.simx_opmode_blocking)
        # ------------------------------------------------------------------------------------------------------------------------------------------

        # Starting simulation----------------------------------------------------------------------------------------------------------------------
        if self.clientID != -1:
            self.start_simulation()
            print('Connected to remote API server')
        else:
            print('Failed connecting to remote API server')
            
    def norm(self,vector):
        sum = 0
        for i in range(len(vector)):
            sum += vector[i] ** 2
        return sum ** 0.5
    
    def read_proximity_sensors(self):
        distances = []
        collide = False
        dis_min = 1
        for i in range(self.number_of_proximity_sensors):
            _, detect, dis, _, _ = vrep.simxReadProximitySensor(self.clientID, self.prox_sensors[i], vrep.simx_opmode_oneshot_wait)
            if detect == 1:
                distances = distances + [self.norm(dis)]
                if distances[i] < 0.1:
                    collide = True
                if distances[i] < dis_min:
                    dis_min = distances[i]
            else:
                distances = distances + [1]  # -1 for previous agents
        return dis_min, collide, distances

    
    # def read_proximity_sensors(self):
    #     distances = []
    #     detections = []
    #     collide = False
    #     dis_min = 1
        
    #     for i in range(self.number_of_proximity_sensors):
    #         _, detect, dis, _, _ = vrep.simxReadProximitySensor(self.clientID, self.prox_sensors[i], vrep.simx_opmode_streaming)
    #         # print(i)
    #         # print(dis)
    #         if detect == 1:
    #             # print(distances)
    #             # distances = distances + [self.norm(dis)]
    #             distances.append(self.norm(dis))
    #             detections.append(1 - ((distances[i] - self.maxDetectionDist) / (self.noDetectionDist - self.maxDetectionDist)))
                
    #             if distances[i] < 0.1:                  
    #                 collide = True
    #             if distances[i] < dis_min:
    #                 dis_min = distances[i]
    #         else:
    #             distances.append(0)
    #             detections.append(0)
                
    #         #     if distances[i] < 0.1:                  
    #         #         collide = True
    #         #     if distances[i] < dis_min:
    #         #         dis_min = distances[i]
    #         # else:
    #         #     distances = distances + [0]  # -1 for previous agents
                    
    #     return dis_min, collide, distances, detections
    
    def set_motor_speeds(self,l_speed, r_speed):
        vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor, l_speed, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor, r_speed, vrep.simx_opmode_streaming)

    def read_robot_position(self):
        return vrep.simxGetObjectPosition(self.clientID, self.Robot, -1, vrep.simx_opmode_blocking)[1]

    
    def read_robot_linear_velocity(self):
        _, velocity, _ = vrep.simxGetObjectVelocity(self.clientID, self.Robot, vrep.simx_opmode_blocking)
        return velocity[0:2]
    
    def get_robot_linear_speed(self,velocity):
        # velocity = read_robot_linear_velocity()
        return (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5
    
    def check_for_terminal_state(self,collided,dis_min):
        # print(collided)
        if collided == True:
            return 2
        # elif dis_min < 0.3:
        #     return 1
        else:
            return 0
    
    def observe(self):
        dis_min, collided, distances = self.read_proximity_sensors()

        # if dis_min<0.3:collided = True
        terminal_state = self.check_for_terminal_state(collided,dis_min)
        
        # state=distances
        state = distances
        # c_state = [angle, target_distance]

        return  dis_min, collided,terminal_state,state
    
    def reset(self):
        
        self.time_without_collision = 0 #stop timer counter
        # Stop the current simulation
        self.stop_simulation()

        # Add a short delay to ensure the simulation stops completely
        time.sleep(0.5)

        # Start a new simulation
        self.start_simulation()

        # Reinitialize or reset any necessary variables or states
        # For example, reset the robot's position or sensors
        initial_sensor_readings = self.read_proximity_sensors()

        # Return the initial state based on sensor readings or other required information
        return initial_sensor_readings
        
    
    def reward_function(self,state, action, next_state, collide, dis_min):

    #Reward function for the P3DX robot.

    # :param state: Current state of the robot.
    # :param action: Action taken by the robot.
    # :param next_state: State of the robot after taking the action.
    # :param collide: Boolean indicating if the robot has collided.
    # :param dis_min: Minimum distance to an obstacle.

    # :return: A numerical reward.

    # Parameters for the reward function, adjust as needed
        collision_penalty = -100
        safe_distance_reward = 10
        distance_penalty = -1
        action_penalty = -0.1 * abs(action)  # Penalize too large actions

        if collide:
            # Heavy penalty for collision
            return collision_penalty

        reward = 0
        if dis_min > 0.3:  # Define this threshold based on your environment
            reward += safe_distance_reward

        # Penalize getting too close to obstacles
        reward += distance_penalty * (1 - dis_min)

        # Penalize large actions
        reward += action_penalty

        return reward

    def reward_function2(self, state, action, next_state, collide, dis_min, time_without_collision):
    # """
    # Reward function for a wall-following robot.

    # :param self: Reference to the environment class instance.
    # :param state: Current state of the robot.
    # :param action: Action taken by the robot.
    # :param next_state: State of the robot after taking the action.
    # :param collide: Boolean indicating if the robot has collided.
    # :param dis_min: Minimum distance to the wall.
    # :param time_without_collision: Time (or timesteps) passed without colliding.

    # :return: A numerical reward.
    # """
    # Constants for reward calculation
        COLLISION_PENALTY = -100  # Penalty for colliding with the wall
        OPTIMAL_DISTANCE = 0.5   # Optimal distance from the wall (meters)
        DISTANCE_REWARD_SCALE = 10.0  # Reward scaling factor for maintaining optimal distance
        ACTION_PENALTY_SCALE = 0.01  # Penalty scaling factor for large actions
        TIME_REWARD_SCALE = 1.0  # Reward scaling factor for time without collision

    # Collision penalty
        if collide:
            return COLLISION_PENALTY

    # Reward for maintaining optimal distance to the wall
        distance_reward = DISTANCE_REWARD_SCALE * (1 - abs(dis_min - OPTIMAL_DISTANCE))

    # Penalty for large actions to encourage smooth movements
        action_penalty = -ACTION_PENALTY_SCALE * abs(action)

    # Time-based reward for navigating without colliding
        time_reward = TIME_REWARD_SCALE * time_without_collision

    # Total reward
        total_reward = distance_reward + action_penalty + time_reward

        return total_reward




    # def step(self,act,p_dis_min):
        
    #     done = False
    #     # print("action-",act)
    #     lmotor = 1-(act/2)
    #     rmotor = 1+(act/2)
    #     self.set_motor_speeds(lmotor,rmotor)
    #     time.sleep(0.6)

    #     dis_min, collided,terminal_state,state = self.observe()
        
        
    #     #This part of the reward function was to keep the promote the robot straight
    #     #But it gives a error of different data types. It actually does give negative numbers as an array, Could not find why.
    #     # print("l-",lmotor,"  r-",rmotor)
    #     motor_dif = abs(lmotor-rmotor)
    #     # if motor_dif < 0.01:
    #     #     motor_dif = 0.01
        
    #     # if motor_dif <= 0.1:
    #     #     motor_dif_reward = 10.0
    #     # else: motor_dif_reward = ((-11.11)*motor_dif) + 1.11   
    #     # print("rdiff-",motor_dif_reward)
    # if terminal_state >0:
    #         reward = -5.0
    #         done = True
    #     else: reward = 30.0*(dis_min-p_dis_min) 
    #     # + motor_dif_reward; print("reward-",reward)
    #     # print(state)

    #     return state, reward, done , dis_min

    def step(self, action):
    # Store the current state
        dis_min, collided, _, current_state = self.observe()
        if collided:
            self.time_without_collision = 0
        else:
            self.time_without_collision += 1
    # Assuming action is a single float value that adjusts motor speeds
        lmotor_speed = 1 - (action / 2)  # Adjust as per your dynamics
        rmotor_speed = 1 + (action / 2)  # Adjust as per your dynamics

    # Set the motor speeds
        self.set_motor_speeds(lmotor_speed, rmotor_speed)

    # Read the next state after performing the action
        next_state = self.read_proximity_sensors()

    # Calculate reward using current and next state
        reward = self.reward_function2(current_state, action, next_state, collided, dis_min, self.time_without_collision)

    # Check if the episode is done
        done = collided  # Example condition, adjust as needed

    # Return the next_state, reward, done, and any additional info
        return next_state, reward, done, {}
  # Additional info can be returned if needed

        
        
        