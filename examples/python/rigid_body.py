from cmath import pi
import sys, time
import numpy as np

# import rigid body and quaternion_control from pyadcs
from libRigidBody import RigidBody
from libController import quaternion_control

# create RigidBody object instance
body = RigidBody()

# pretty print states
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

def main():

    # controller stiffness gain
    K = 10.0

    # controller damping gain
    C = 1.0

    # time
    t = 0
    delta_t = 0.01

    # set desired state vector in form of numpy array [qw qx qy qz wx wy wz]
    DESIRED_STATE = np.array(
        [np.cos(45*pi/180.), 0., 0., np.sin(45*pi/180.), 0., 0., 0.])

    t_next = time.time() + 0.01

    while(True):

        try:
            # get current body state
            state = body.get_state()

            # compute torques with quaternion feedback control
            action = quaternion_control(state, DESIRED_STATE, K, C)

            # integrate step in time with control action
            state = body.step(action, t, t+delta_t, delta_t/10.)

            # pretty print states
            if(t_next< time.time() + 0.025):
                print("Quaternion :" + str(state[0:4]))
                print("Rate       :" + str(state[4:7])+"\n")
                t_next = t_next + 0.025

            # increament time
            t = t + delta_t

        except KeyboardInterrupt:
            # quit
            sys.exit()


if __name__ == "__main__":
    main()
