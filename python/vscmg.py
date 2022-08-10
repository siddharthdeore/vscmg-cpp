from cmath import pi
import sys
import numpy as np

# import rigid body and quaternion_control from pyadcs
from pyadcs import VSCMG, quaternion_control

# create VSCMG object instance
body = VSCMG()


def main():

    # controller stiffness gain
    K = 100.0

    # controller damping gain
    C = 10.0

    # time
    t = 0.0
    delta_t = 0.01

    # set desired state vector in form of numpy array [qw qx qy qz wx wy wz delta and omega to be removed later]
    desired_state = np.array([np.cos(45*pi/180.), 0., 0., np.sin(45*pi/180.), 0.,0.,0.])

    while(True):

        try:
            # get current body state
            state = body.get_state()

            # compute torques with quaternion feedback control
            torque = quaternion_control(state, desired_state, K, C)

            # compute required actuator signals from given torque
            action = body.calc_steering(torque,0.0)

            # integrate step in time with control action
            state = body.step(action, t, t+delta_t, delta_t/10.)

            # pretty print states
            with np.printoptions(precision=4, suppress=True):
                print("Quaternion :" + str(state[0:4]))
                print("Rate       :" + str(state[4:7]))
                print("Delta      :" + str(state[7:11]))
                print("Omega      :" + str(state[11:15]) + "\n")

            # increament time
            t = t + delta_t

        except KeyboardInterrupt:
            # quit
            sys.exit()


if __name__ == "__main__":
    main()
