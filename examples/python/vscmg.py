from cmath import pi
import sys, time
import numpy as np

# import rigid body and quaternion_control from pyadcs
from adcs import VSCMG
from adcs import quaternion_control

# create VSCMG object instance
body = VSCMG()

# pretty print states
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qw, qx, qy, qz]

def main():

    # controller stiffness gain
    K = 1.0

    # controller damping gain
    C = 10.

    # time
    t = 0.0
    delta_t = 0.01
    deg = pi/180.
    quat = get_quaternion_from_euler(0*deg,0*deg,180*deg)
    w = [0.,0.,0.]
    # set desired state vector in form of numpy array [qw qx qy qz wx wy wz]
    desired_state = np.array(quat+w)
    body.set_inertia(1.0,2.0,3.0,0.0,0.0,0.0)

    t_next = time.time() + 0.01

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

            if(t_next< time.time() + 0.025):
                print("Quaternion :" + str(state[0:4]))
                print("Rate       :" + str(state[4:7]))
                print("Delta      :" + str(state[7:11]))
                print("Omega      :" + str(state[11:15]) + "\n")
                t_next = t_next + 0.025

            # increament time
            t = t + delta_t

        except KeyboardInterrupt:
            # quit
            sys.exit()


if __name__ == "__main__":
    main()
