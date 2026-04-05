import math
import numpy as np

T1 = math.radians(50)
T2 = math.radians(32)
T3 = math.radians(8)
T4 = math.radians(2)


a1 = 16.4
a2 = 10.5
a3 = 14.8
a4 = 18.0


DH_PARAMS = [
    [T1,math.radians(90), 0, a1],
    [T2,math.radians(180), a2, 0],
    [T3 + math.radians(90),0, -a3, 0],
    [T4 + math.radians(90),0, a4, 0],
]

def dh_matrix(theta, alpha, r, d):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(dh_params):
    H0_1 = dh_matrix(*dh_params[0])
    H1_2 = dh_matrix(*dh_params[1])
    H2_3 = dh_matrix(*dh_params[2])
    H3_4 = dh_matrix(*dh_params[3])

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)
    H0_4 = np.dot(H0_3, H3_4)
    x = H0_4[0, 3]
    y = H0_4[1, 3]
    z = H0_4[2, 3]
    return x, y, z

if __name__ == "__main__":
    x, y, z = forward_kinematics(DH_PARAMS)
    print(f"End Effector Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")