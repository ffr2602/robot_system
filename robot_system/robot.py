from math import pi
import numpy as np

class robot:
    def __init__(self, GEOMETRI_ROBOT, WHEEL_RADIUS):

        bb = 1/GEOMETRI_ROBOT

        Ti = np.array(
            [[1,-1,-GEOMETRI_ROBOT],
             [1, 1, GEOMETRI_ROBOT],
             [1, 1,-GEOMETRI_ROBOT],
             [1,-1, GEOMETRI_ROBOT]]
        )

        Tf = np.array(
            [[  1,  1,  1,  1],
             [ -1,  1,  1, -1],
             [-bb, bb,-bb, bb]]
        )
        
        self.inverse_transform_matrix = (1/WHEEL_RADIUS)*Ti
        self.forward_transform_matrix = (WHEEL_RADIUS/4)*Tf

    def compute_velocity_robot_inverse_kinematic(self, input):
        motor_velocity = np.zeros(4).astype(int)
        robot_velocity = np.array([input[0], input[1], input[2]])
        raw_data = np.matmul(self.inverse_transform_matrix, robot_velocity)
        for i in range(len(raw_data)):
            motor_velocity[i] = int(self.convert_rad_to_RPM(raw_data[i]))
        return motor_velocity
    
    def compute_velocity_robot_forward_kinematic(self, input):
        twist_velocity = np.zeros(3)
        motor_velocity = self.velocity_rad(input)
        raw_data = np.matmul(self.forward_transform_matrix, motor_velocity)
        for i in range(len(raw_data)):
            twist_velocity[i] = raw_data[i]
        return twist_velocity
    
    def velocity_rad(self, input):
        motor_velocity = np.array([self.convert_RPM_to_rad(input[0]), self.convert_RPM_to_rad(input[1]), self.convert_RPM_to_rad(input[2]), self.convert_RPM_to_rad(input[3])])
        return motor_velocity

    def convert_rad_to_RPM(self, input):
        return input * 60 / (2 * pi)
    
    def convert_RPM_to_rad(self, input):
        return input * 2 * pi / 60