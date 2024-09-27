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
    robot.setVelocity(0.5)

    # move robot to home position
    robot.home()

    home_affine = robot.get_transform('fanuc_m16ib_tool0', 'fanuc_m16ib_base_link', affine=True)
    print(f'home_affine: {home_affine}')

    for i in range(100):
        # move robot to pre-pick
        prepick_affine = Affine((-0.0009161331690847874, 1.0161885738372803, 0.8262154459953308), (0.0001285069592995569, 0.9999606013298035, 0.008876869454979897, -3.314939749543555e-05))
        feedback = robot.ptp(prepick_affine)
        print('Motion Feedback Pre-Pick:', feedback)


        rel_motion = Affine((0.0, 0.0, 0.3))    # in tcp koordinates
        pick_affine = prepick_affine * rel_motion
        feedback_pick = robot.lin(pick_affine)
        print('Motion Feedback Pick:', feedback_pick)


        place_affine = Affine((0.9930514693260193, 1.0438474416732788, 0.7302162051200867), (0.37387216091156006, 0.9274460673332214, 0.007607490289956331, -0.002341807819902897))
        feedback_place = robot.ptp(place_affine)
        print('Motion Feedback Place:', feedback_place)
        
        # move robot to home position
        feedback_home = robot.home()
        print('Motion Feedback Home:', feedback_home)

    # move robot to home position
    feedback_home = robot.home()

    # destroy the robot node, stop execution
    robot.destroy_node()

    # shutdown previously initialized context
    rclpy.shutdown()


if __name__ == '__main__':
    main()