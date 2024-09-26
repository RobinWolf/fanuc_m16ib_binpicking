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

    #define a home position (due to linked joint 2 and 3, there is an additional virtual joint (i=2) which has to be 0.0 all the time !!!
    robot.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.setVelocity(0.2)

    # move robot to home position
    robot.home()

    home_affine = robot.get_transform('fanuc_m16ib_tool0', 'fanuc_m16ib_base_link', affine=True)
    print(f'home_affine: {home_affine}')

    # move robot to a specific position
    # test_affine = Affine((-1.08062555e-03,  1.0790512561798096,  0.9635626673698425), (2.4000075427466072e-05,  0.9999187588691711,  0.01274777390062809,  -1.599685310793575e-05))
    test_affine = Affine((0.5500411987304688, 1.0699824094772339, 0.4789556860923767), (2.2131800836433513e-09, 0.7069438099861145, 0.7072696685791016 ,9.916898852679878e-05))
    # print('Test Affine:', test_affine)
    # print('Test_Affine_Translation:', test_affine.translation)
    # print('Test_Affine_Quaternion:', test_affine.quat)

    feedback = robot.ptp(test_affine)
    print('next pose')
    #feedback = robot.ptp_joint([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    print('Motion Feedback:', feedback)

    # move robot to home position
    # robot.home()


    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()