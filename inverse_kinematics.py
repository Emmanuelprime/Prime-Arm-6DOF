import math
import numpy as np

a1 = 16.4
a2 = 10.5
a3 = 14.8
a4 = 18.0


def dh_matrix(theta, alpha, r, d):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def forward_kinematics(T1, T2, T3, T4):
    DH_PARAMS = [
        [T1, math.radians(90), 0, a1],
        [T2, math.radians(180), a2, 0],
        [T3 + math.radians(90), 0, -a3, 0],
        [T4 + math.radians(90), 0, a4, 0],
    ]

    H = np.eye(4)
    for p in DH_PARAMS:
        H = H @ dh_matrix(*p)

    return H[0, 3], H[1, 3], H[2, 3]


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def inverse_kinematics_numeric(x_target, y_target, z_target):
    T = np.array([0.0, 0.0, 0.0, 0.0])  # initial guess

    for i in range(200):
        x, y, z = forward_kinematics(*T)

        error = np.array([x_target - x, y_target - y, z_target - z])

        if np.linalg.norm(error) < 1e-3:
            break

        # Jacobian
        J = np.zeros((3, 4))
        delta = 1e-5

        for j in range(4):
            T_temp = T.copy()
            T_temp[j] += delta
            x1, y1, z1 = forward_kinematics(*T_temp)

            J[:, j] = [(x1 - x)/delta, (y1 - y)/delta, (z1 - z)/delta]

        # 🔥 KEY FIX: pseudo-inverse instead of transpose
        J_pinv = np.linalg.pinv(J)

        dT = J_pinv @ error

        # Limit step size (VERY IMPORTANT)
        dT = np.clip(dT, -0.1, 0.1)

        T += dT

        # Normalize angles every step
        T = np.array([normalize_angle(t) for t in T])

    return T


# =========================
# TEST
# =========================
if __name__ == "__main__":
    x, y, z = -0.88, 0.85, 56.47

    T = inverse_kinematics_numeric(x, y, z)

    print("\nSolved Angles (degrees):")
    print(f"T1={math.degrees(T[0]):.2f}")
    print(f"T2={math.degrees(T[1]):.2f}")
    print(f"T3={math.degrees(T[2]):.2f}")
    print(f"T4={math.degrees(T[3]):.2f}")

    # Verify
    xf, yf, zf = forward_kinematics(*T)
    print("\nFK Check:")
    print(f"x={xf:.2f}, y={yf:.2f}, z={zf:.2f}")