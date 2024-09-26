import rclpy
import numpy as np
import time
from api_moveit_clients.transform import Affine #for 6D Transformation 

#import clients // python api
from api_moveit_clients.api_moveit import ARMClient


def main():
    # initialize ros communications for a given context 
    rclpy.init(args=None)

    # initialize/ bring up node with agv clients
    robot = ARMClient()

    #define a home position (when want to use default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] you don't need this definition) -> floats required
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.setVelocity(0.2)

    # move robot to home position
    robot.home()

    for i in range(10):
        home_affine = robot.get_transform('fanuc_m16ib_tool0', 'fanuc_m16ib_base_link', affine=True)
        print(f'home_affine: {home_affine}')
    
        # move robot to a specific position
        test_affine = Affine((0.653, 1.234, 0.766), (0.241, 0.969, 0.038 ,0.009))
        print('Test Affine:', test_affine)
        print('Test_Affine_Translation:', test_affine.translation)
        print('Test_Affine_Quaternion:', test_affine.quat)

        feedback = robot.ptp(test_affine)
        
        print('Feedback:', feedback)

        # move robot to home position
        robot.home()


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()