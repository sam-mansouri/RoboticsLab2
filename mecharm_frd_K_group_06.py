import numpy as np
from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

# -------------------------------------------------
# 1. Define symbolic joint variables
# -------------------------------------------------
q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

# -------------------------------------------------
# 2. Helper function: DH Transformation Matrix
# -------------------------------------------------
def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])
    return M

# -------------------------------------------------
# 3. Define D-H Table for MechArm 270 Pi
# NOTE: Replace these with the REAL robot parameters
# Each row: [a, alpha, d, theta]
# -------------------------------------------------
DH_table = [
    [0,      pi/2,   d1, q1],
    [a2,     0,      0,  q2],
    [a3,     0,      0,  q3],
    [0,      pi/2,   d4, q4],
    [0,     -pi/2,   d5, q5],
    [0,      0,      d6, q6]
]

# You must define real numeric values for a2, a3, d1, d4, d5, d6
# Example (FAKE values, replace with real ones):
d1 = 100
a2 = 100
a3 = 100
d4 = 80
d5 = 60
d6 = 50

# Rebuild table with numbers
DH_table = [
    [0,      pi/2,   d1, q1],
    [a2,     0,      0,  q2],
    [a3,     0,      0,  q3],
    [0,      pi/2,   d4, q4],
    [0,     -pi/2,   d5, q5],
    [0,      0,      d6, q6]
]

# -------------------------------------------------
# 4. Compute individual transformation matrices
# -------------------------------------------------
T_matrices = []

for row in DH_table:
    a, alpha, d, theta = row
    T = get_transformation_matrix(a, alpha, d, theta)
    T_matrices.append(T)

# -------------------------------------------------
# 5. Compute overall transformation T0_6
# -------------------------------------------------
T_total = T_matrices[0]
for i in range(1, len(T_matrices)):
    T_total = T_total * T_matrices[i]

T_total = simplify(T_total)

print("Symbolic T0_6:")
print(T_total)

# -------------------------------------------------
# 6. Joint angle offsets (one joint has ±90° offset if needed)
# -------------------------------------------------
# Example: suppose joint 2 has -90 degree offset
offsets = [0, -pi/2, 0, 0, 0, 0]

# -------------------------------------------------
# 7. Substitute numeric joint angles
# -------------------------------------------------
def forward_kinematics(joint_angles_deg):
    # Convert degrees to radians
    joint_angles = [np.radians(a) for a in joint_angles_deg]

    subs_dict = {
        q1: offsets[0] + joint_angles[0],
        q2: offsets[1] + joint_angles[1],
        q3: offsets[2] + joint_angles[2],
        q4: offsets[3] + joint_angles[3],
        q5: offsets[4] + joint_angles[4],
        q6: offsets[5] + joint_angles[5],
    }

    T_num = T_total.subs(subs_dict)
    T_num = simplify(T_num)

    # Extract end-effector position (x, y, z)
    x = T_num[0, 3]
    y = T_num[1, 3]
    z = T_num[2, 3]

    return float(x), float(y), float(z), T_num

# -------------------------------------------------
# 8. Test with neutral position [0,0,0,0,0,0]
# -------------------------------------------------
if __name__ == "__main__":
    test_angles = [0, 0, 0, 0, 0, 0]
    x, y, z, T = forward_kinematics(test_angles)

    print("End effector position at zero pose:")
    print("x =", x, "y =", y, "z =", z)
    print("Full T matrix:")
    print(T)
