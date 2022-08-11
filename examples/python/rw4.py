from cmath import pi
import sys
import numpy as np

# import rigid body and quaternion_control from pyadcs
from libRW4 import RW4
from libController import quaternion_control

# create RW4 object instance
body = RW4()


def main():

    # controller stiffness gain
    K = 1000.0

    # controller damping gain
    C = 100.0

    # time
    t = 0.0
    delta_t = 0.01

    # set desired state vector in form of numpy array [qw qx qy qz wx wy wz]
    desired_state = np.array(
        [np.cos(45*pi/180.), 0., 0., np.sin(45*pi/180.), 0., 0., 0.])
    i = 0
    while(True):

        try:
            # get current body state
            state = body.get_state()

            # compute torques with quaternion feedback control
            torque = quaternion_control(state, desired_state, K, C)

            # compute required actuator signals from given torque
            action = body.calc_steering(torque, 0.0)

            # integrate step in time with control action
            state = body.step(action, t, t+delta_t, delta_t/10.)
            if(i%100 == 0):
                # pretty print states
                with np.printoptions(precision=4, suppress=True):
                    print("Quaternion :" + str(state[0:4]))
                    print("Rate       :" + str(state[4:7]))
                    print("Omega      :" + str(state[7:11]) + "\n")

            # increament time
            t = t + delta_t
            i = i + 1

        except KeyboardInterrupt:
            # quit
            sys.exit()


if __name__ == "__main__":
    main()
