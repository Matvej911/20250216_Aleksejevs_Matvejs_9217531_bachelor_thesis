import sys
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

import serial
import time


# Serial initialization
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  


def send_angles(angles):
    """Send joint angles to Arduino via Serial."""
    data_string = ",".join([f"{angle:.2f}" for angle in angles]) + "\n"  # Format angles with 2 decimals
    ser.write(data_string.encode())  # Send as bytes
    print("Sent angles:", data_string.strip())

def run_robotics_script(coordinates):
    # Unpack the coordinates
    x1, y1, z1 = coordinates

    print('COORDINATES',coordinates)

    modifiedrobot = rtb.ERobot.URDF('robot_file.urdf')
    ets = modifiedrobot.ets()


    euler_x1, euler_y1, euler_z1 = 0, 0, -185  # Initial orientation (roll, pitch, yaw)

    # Convert initial Euler angles to a transformation matrix
    R1 = SE3.RPY([np.radians(euler_x1), np.radians(euler_y1), np.radians(euler_z1)], order='xyz')  # Rotation matrix
    Tep1 = SE3(x1, y1, z1) * R1  # Combine position and orientation into a transformation matrix

    # Solve the IK problem to find joint angles for the initial pose
    initial_guess = ets.ikine_LM(ets.fkine(np.zeros(ets.n)), method='wampler').q
    joint_angles1, *_ = ets.ik_LM(Tep1, q0=initial_guess, ilimit=200, slimit=3000, tol=1e-5, method='chan', k=0.1, mask=[1, 1, 1, 1, 0, 0])

    # Convert joint angles to degrees for printing
    joint_angles_degrees1 = np.degrees(joint_angles1)
    print("Initial Joint Angles (Degrees):", joint_angles_degrees1)  # These angles should be sent


    # Return joint angles for further use
    send_angles(joint_angles_degrees1)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        coordinates = eval(sys.argv[1])
        run_robotics_script(coordinates)
