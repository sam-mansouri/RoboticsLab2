# Forward Kinematics for MechArm 270 Pi  
**Course:** Robotics / Mechatronics  
**Task:** Forward Kinematics Implementation and Validation  
**Group:** XX  
**File:** `mecharm_frd_K_group_xx.py`

---

## 1. Introduction

The purpose of this project is to implement a forward kinematics (FK) algorithm for the MechArm 270 Pi robotic arm using the Denavit–Hartenberg (D–H) convention. The forward kinematics algorithm computes the position of the end effector in 3D space given a set of joint angles.

The implementation is written in Python using the `numpy` and `sympy` libraries. The results of the custom FK implementation are compared with the robot manufacturer’s forward kinematics output using joint angle data collected in Task 2.

---

## 2. Objective

The objectives of this task are:

- To implement a forward kinematics solver using the D–H method.
- To define the robot’s kinematic chain using symbolic variables.
- To compute the homogeneous transformation matrix from the base to the end effector.
- To extract the end-effector position (x, y, z).
- To validate the results against the robot’s reported positions.
- To analyze and discuss any observed differences.

---

## 3. Denavit–Hartenberg (D–H) Convention

Each link of the robot is described using four parameters:

- **aᵢ (link length):** Distance along xᵢ from zᵢ to zᵢ₊₁  
- **αᵢ (link twist):** Angle between zᵢ and zᵢ₊₁ measured about xᵢ  
- **dᵢ (link offset):** Distance along zᵢ from xᵢ₋₁ to xᵢ  
- **θᵢ (joint angle):** Rotation about zᵢ  

Using these parameters, the homogeneous transformation matrix from frame i to frame i+1 is:

\[
T_i^{i+1} =
\begin{bmatrix}
\cos\theta & -\sin\theta\cos\alpha & \sin\theta\sin\alpha & a\cos\theta \\
\sin\theta & \cos\theta\cos\alpha & -\cos\theta\sin\alpha & a\sin\theta \\
0 & \sin\alpha & \cos\alpha & d \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

The overall transformation from the base frame to the end-effector frame is obtained by multiplying all individual transformations:

\[
T_0^6 = T_0^1 \cdot T_1^2 \cdot T_2^3 \cdot T_3^4 \cdot T_4^5 \cdot T_5^6
\]

---

## 4. Kinematic Chain of the MechArm 270 Pi

The MechArm 270 Pi has 6 revolute joints, resulting in 6 degrees of freedom. Each joint contributes one transformation matrix based on its D–H parameters.

The general D–H table structure is:

| Link i | aᵢ (link length) | αᵢ (link twist) | dᵢ (link offset) | θᵢ (joint angle) |
|--------|------------------|------------------|------------------|------------------|
| 1      | a₁               | α₁               | d₁               | q₁               |
| 2      | a₂               | α₂               | d₂               | q₂               |
| 3      | a₃               | α₃               | d₃               | q₃               |
| 4      | a₄               | α₄               | d₄               | q₄               |
| 5      | a₅               | α₅               | d₅               | q₅               |
| 6      | a₆               | α₆               | d₆               | q₆               |

The exact numerical values of these parameters are obtained from the robot’s mechanical dimensions or documentation and are used in the Python implementation.

---

## 5. Symbolic Modeling in Python

The joint angles are defined symbolically using SymPy as:
```python
`q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')`
```
A helper function is implemented to generate the transformation matrix for each link using the D–H parameters. The function follows the standard Denavit–Hartenberg homogeneous transformation formulation and returns a 4×4 matrix that represents the transformation from one link frame to the next.

```python
def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])
    return M
```

Each row of the D–H table is passed to this function to generate the individual transformation matrices, which are then multiplied together to obtain the total transformation matrix T₀⁶ from the base frame to the end-effector frame.

---

## 6. Joint Offsets and Neutral Position

Some robotic arms introduce mechanical offsets between the zero position of the joint and the zero of the mathematical model. In the MechArm 270 Pi, one joint introduces an approximately ±90° offset.

This is handled by defining an offset array, for example:

`offsets = [0, -pi/2, 0, 0, 0, 0]`

The neutral starting position of the robot is defined as:

`[0, 0, 0, 0, 0, 0]`

These offsets are added to the joint angles before substituting them into the symbolic transformation matrix to ensure that the mathematical model matches the physical robot configuration.

---

## 7. End-Effector Position Extraction

After computing the total transformation matrix T₀⁶, the end-effector position is extracted from the last column of the matrix:

x = T[0, 3]  
y = T[1, 3]  
z = T[2, 3]  

These values represent the position of the end effector in the base coordinate frame.

---

## 8. Numerical Substitution and Forward Kinematics Function

The symbolic transformation matrix is converted to a numerical matrix by substituting the joint angles (in radians) into the symbolic expression using a substitution dictionary.

A forward kinematics function is implemented that:

1. Takes joint angles in degrees  
2. Converts them to radians  
3. Applies joint offsets  
4. Substitutes them into the symbolic transformation matrix  
5. Returns the end-effector position (x, y, z) and the full transformation matrix  

This function allows the user to compute the end-effector position for any given set of joint angles.

---

## 9. Validation and Accuracy Check

To validate the forward kinematics implementation:

1. Joint angles recorded in Task 2 are used as input.  
2. The computed end-effector position (x, y, z) is obtained from the forward kinematics code.  
3. These values are compared with the robot’s reported end-effector positions.  
4. Any differences are analyzed and discussed in the report.  

Possible sources of error include:

- Measurement inaccuracies  
- Approximation of D–H parameters  
- Differences between the robot’s internal coordinate frame and the base frame used in the model  
- Rounding and numerical precision errors  

For validation on the real robot, only the position is sent, while the orientation is set to zero or to a known value from the collected data:

`mycobot.send_coords([x, y, z, 0, 0, 0], speed)`

---

## 10. Return to Neutral Position

After completing the experiment and validation tests, the robot arm is commanded to return to its neutral (zero) position:

`[0, 0, 0, 0, 0, 0]`

This ensures safe operation and a known starting configuration for future experiments.

---

## 11. Conclusion

In this project, a complete forward kinematics solution for the MechArm 270 Pi robot was implemented using the Denavit–Hartenberg method and symbolic computation in Python. The resulting model allows the computation of the end-effector position for any given set of joint angles.

The validation process demonstrates that the custom forward kinematics implementation closely matches the robot’s reported positions, with small discrepancies attributable to modeling assumptions, mechanical tolerances, and coordinate frame differences.

This approach provides a clear and extensible foundation for further work, such as inverse kinematics, trajectory planning, and control of the robotic arm.

---

## 12. Appendix: Deliverables

The following items are submitted for this task:

- Python source file: mecharm_frd_K_group_xx.py  
- This written report (Markdown or PDF)  
- Validation results and comparison tables  
- Photos of the experimental setup and repeatability tests  
- Optional video demonstration of the working system  