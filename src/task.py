import sympy as sp

# Define symbolic variables for joint angles and DH parameters
a1, a2, a3, a4, a5, a6 = sp.symbols('a1 a2 a3 a4 a5 a6')
d1, d2, d3, d4, d5, d6 = sp.symbols('d1 d2 d3 d4 d5 d6')
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha1 alpha2 alpha3 alpha4 alpha5 alpha6')
theta1, theta2, theta3, theta4, theta5, theta6 = sp.symbols('theta1 theta2 theta3 theta4 theta5 theta6')

# Task 1: Symbolic Expression of T0^5
def dh_transform_matrix(a, alpha, d, theta):
    """Create a DH transformation matrix for a single joint."""
    ct = sp.cos(theta)
    st = sp.sin(theta)
    ca = sp.cos(alpha)
    sa = sp.sin(alpha)
    
    return sp.Matrix([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

# Create transformation matrices for each joint
T01 = dh_transform_matrix(0, alpha1, 0, theta1)
T12 = dh_transform_matrix(a2, 0, 0, 0)
T23 = dh_transform_matrix(a3, alpha3, 0, theta3)
T34 = dh_transform_matrix(0, alpha4, d4, 0)
T45 = dh_transform_matrix(0, alpha5, 0, theta5)

# Compute T0^5
T05 = T01 * T12 * T23 * T34 * T45

# Extract wrist center position from T05
p_wx = T05[0, 3]
p_wy = T05[1, 3]
p_wz = T05[2, 3]

# Execute all tasks directly
if __name__ == "__main__":
    print("="*60)
    print("Lab 8: Inverse Kinematics Solution with Symbolic DH Parameters")
    print("="*60)
    
    # Task 1: Output wrist center position
    print("\nTask 1: Wrist center position (from T05):")
    print("p_wx = ")
    sp.pprint(p_wx)
    print("\np_wy = ")
    sp.pprint(p_wy)
    print("\np_wz = ")
    sp.pprint(p_wz)
    
    # Task 2: Simplify squared distance and show equation for θ3
    squared_dist = p_wx**2 + p_wy**2 + p_wz**2
    simplified_dist = sp.simplify(squared_dist)
    D_sq = sp.symbols('D_sq')
    
    print("\n" + "="*60)
    print("Task 2: Squared distance from wrist center to origin:")
    print("Simplified squared distance:")
    sp.pprint(simplified_dist)
    print("\nEquation to solve for θ3:")
    sp.pprint(sp.Eq(simplified_dist, D_sq))
    
    # Task 3: Simplify XY projection
    xy_squared = p_wx**2 + p_wy**2
    simplified_xy = sp.simplify(xy_squared)
    
    print("\n" + "="*60)
    print("Task 3: Squared projection on XY-plane:")
    print("Expression relates to θ2 + θ3:")
    sp.pprint(simplified_xy)
    
    # Task 4: Solution for θ1
    print("\n" + "="*60)
    print("Task 4: Solution for θ1:")
    print("θ1 = atan2(p_wy, p_wx)")
    print("\nSymbolic expression:")
    sp.pprint(sp.atan2(p_wy, p_wx))
    
    # Task 5: Compute T3^6 and analyze orientation
    # Compute T0^3 (first three joints)
    T03 = T01 * T12 * T23
    
    # Create T06 (full transformation)
    T56 = dh_transform_matrix(0, 0, d6, 0)
    T06 = T05 * T56
    
    # Compute T3^6 = (T0^3)^{-1} * T0^6
    T03_inv = T03.inv()
    T36 = T03_inv * T06
    
    # Extract rotation matrix from T36
    R36 = T36[0:3, 0:3]
    
    print("\n" + "="*60)
    print("Task 5: T3^6 rotation matrix:")
    print("This matrix contains the orientation information for θ4, θ5, θ6")
    print("\nRotation matrix R36:")
    sp.pprint(R36)
    
    print("\nSolution approach for spherical wrist:")
    print("θ5 = atan2(sqrt(r13^2 + r33^2), r23)")
    print("θ4 = atan2(r33/sin(θ5), -r13/sin(θ5))")
    print("θ6 = atan2(-r22/sin(θ5), r21/sin(θ5))")
    
    # Task 6: Spherical wrist analysis
    print("\n" + "="*60)
    print("Task 6: Analysis of spherical wrist characteristics")
    print("""
Characteristics of a Spherical Wrist and Two Solutions:

A spherical wrist has its last three joint axes (θ4, θ5, θ6) intersecting at a single point (the wrist center).
This configuration provides decoupled position and orientation control:
- Position is controlled by the first three joints (θ1, θ2, θ3)
- Orientation is controlled by the last three joints (θ4, θ5, θ6)

The existence of two solutions for the spherical wrist comes from the nature of the inverse trigonometric functions:
1. The solution for θ5 has two possibilities: 
   θ5 ∈ (0, π)  (elbow up configuration)
   θ5 ∈ (-π, 0) (elbow down configuration)

2. For each θ5 solution, there is a corresponding pair (θ4, θ6) that gives the same orientation.

This leads to two distinct configurations for the wrist:
- One with positive θ5 (wrist bent in one direction)
- One with negative θ5 (wrist bent in the opposite direction)

The two solutions provide flexibility for the robot to reach the same end-effector pose with different joint 
configurations, which can be useful for avoiding obstacles or optimizing movement paths.
    """)
    
    print("\n" + "="*60)
    print("All tasks completed!")