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

    def step(self,act,p_dis_min):
        
        done = False
        # print("action-",act)
        lmotor = 1-(act/2)
        rmotor = 1+(act/2)
        self.set_motor_speeds(lmotor,rmotor)
        time.sleep(0.6)

        dis_min, collided,terminal_state,state = self.observe()
        
        
        #This part of the reward function was to keep the promote the robot straight
        #But it gives a error of different data types. It actually does give negative numbers as an array, Could not find why.
        # print("l-",lmotor,"  r-",rmotor)
        motor_dif = abs(lmotor-rmotor)
        # if motor_dif < 0.01:
        #     motor_dif = 0.01
        
        # if motor_dif <= 0.1:
        #     motor_dif_reward = 10.0
        # else: motor_dif_reward = ((-11.11)*motor_dif) + 1.11   
        # print("rdiff-",motor_dif_reward)
        
        
        if terminal_state >0:
            reward = -5.0
            done = True
        else: reward = 30.0*(dis_min-p_dis_min) 
        # + motor_dif_reward; print("reward-",reward)
        # print(state)

        return state, reward, done , dis_min