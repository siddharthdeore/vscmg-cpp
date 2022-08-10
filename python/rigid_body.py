from math import pi
import sys
import numpy as np

# import rigid body and quaternion_control from pyadcs
from pyadcs import RigidBody, quaternion_control

# create RigidBody object instance
body = RigidBody()


def main():

    # controller stiffness gain
    K = 10.0

    # controller damping gain
    C = 1.0

    # time
    t = 0
    delta_t = 0.01

    # set desired state vector in form of numpy array [qw qx qy qz wx wy wz]
    DESIRED_STATE = np.array([np.cos(45*pi/180.), 0., 0., np.sin(45*pi/180.), 0., 0., 0.])

    while(True):

        try:
            # get current body state
            state = body.get_state()

            # compute torques with quaternion feedback control
            action = quaternion_control(state, DESIRED_STATE, K, C)

            # integrate step in time with control action
            state = body.step(action, t, t+delta_t, delta_t/10.)

            # pretty print states
            with np.printoptions(precision=4, suppress=True):
                print("Quaternion :" + str(state[0:4]))
                print("Rate       :" + str(state[4:7])+"\n")

            # increament time
            t = t + delta_t

        except KeyboardInterrupt:
            # quit
            sys.exit()


if __name__ == "__main__":
    main()
