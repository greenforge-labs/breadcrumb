#!/usr/bin/env python3
"""
Compute optimal LQR gains for the cartpole system.

This script linearizes the cartpole dynamics around the upright equilibrium
and computes the LQR optimal gain matrix K.
"""

import numpy as np
from scipy import linalg

# Physical parameters (from cartpole_simulator.hpp)
M = 1.0  # Cart mass (kg)
m = 0.1  # Pole mass (kg)
L = 0.5  # Pole length to center of mass (meters) - half of total length
g = 9.81  # Gravity (m/s^2)
b = 0.1  # Friction coefficient

print("=" * 60)
print("LQR Gain Computation for Cartpole System")
print("=" * 60)
print(f"\nPhysical Parameters:")
print(f"  Cart mass (M):  {M} kg")
print(f"  Pole mass (m):  {m} kg")
print(f"  Pole length (L): {L} m (to center of mass)")
print(f"  Gravity (g):    {g} m/s^2")
print(f"  Friction (b):   {b}")

# Linearized state-space model around upright equilibrium
# State: x = [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
# Input: u = force on cart

# A matrix (linearized dynamics)
# Derived from the nonlinear cartpole equations, linearized around theta = 0
A = np.array([[0, 1, 0, 0], [0, -b / M, -m * g / M, 0], [0, 0, 0, 1], [0, b / (M * L), (M + m) * g / (M * L), 0]])

# B matrix (input)
B = np.array([[0], [1 / M], [0], [-1 / (M * L)]])

print("\nState-space matrices:")
print("\nA matrix (state dynamics):")
print(A)
print("\nB matrix (input):")
print(B)

# LQR cost matrices
# Q: Penalize state deviations
# R: Penalize control effort

# Q matrix - penalize deviations in state
# [cart_pos, cart_vel, pole_angle, pole_vel]
Q = np.diag(
    [
        1.0,  # Cart position - keep centered
        0.1,  # Cart velocity - don't care much about speed
        100.0,  # Pole angle - MOST IMPORTANT - keep upright!
        1.0,  # Pole angular velocity - prevent oscillations
    ]
)

# R matrix - penalize control effort
R = np.array([[0.1]])  # Small penalty allows larger control forces

print("\nLQR weighting matrices:")
print("\nQ matrix (state penalty):")
print(Q)
print("\nR matrix (control penalty):")
print(R)

# Solve continuous-time algebraic Riccati equation
try:
    P = linalg.solve_continuous_are(A, B, Q, R)

    # Compute optimal gain matrix K
    K = linalg.inv(R) @ B.T @ P

    print("\n" + "=" * 60)
    print("LQR Solution")
    print("=" * 60)
    print("\nOptimal gain matrix K:")
    print(K)

    k1, k2, k3, k4 = K[0]

    print("\nIndividual gains:")
    print(f"  k1 (cart position):        {k1:8.4f}")
    print(f"  k2 (cart velocity):        {k2:8.4f}")
    print(f"  k3 (pole angle):           {k3:8.4f}")
    print(f"  k4 (pole angular velocity): {k4:8.4f}")

    print("\n" + "=" * 60)
    print("ROS 2 Parameter Commands")
    print("=" * 60)
    print("\nTo set these gains on the running controller:")
    print(f"ros2 param set /cartpole_controller k1 {k1}")
    print(f"ros2 param set /cartpole_controller k2 {k2}")
    print(f"ros2 param set /cartpole_controller k3 {k3}")
    print(f"ros2 param set /cartpole_controller k4 {k4}")

    print("\n" + "=" * 60)
    print("Launch File Format")
    print("=" * 60)
    print("\nFor the launch file, use:")
    print("parameters={")
    print(f'    "k1": {k1},')
    print(f'    "k2": {k2},')
    print(f'    "k3": {k3},')
    print(f'    "k4": {k4},')
    print("}")

    # Verify closed-loop stability
    A_cl = A - B @ K
    eigenvalues = np.linalg.eigvals(A_cl)

    print("\n" + "=" * 60)
    print("Stability Analysis")
    print("=" * 60)
    print("\nClosed-loop eigenvalues (should have negative real parts):")
    for i, eig in enumerate(eigenvalues):
        stability = "STABLE" if eig.real < 0 else "UNSTABLE"
        print(f"  λ{i+1} = {eig.real:8.4f} + {eig.imag:8.4f}j  [{stability}]")

    if all(eig.real < 0 for eig in eigenvalues):
        print("\n✓ System is STABLE with these gains!")
    else:
        print("\n✗ WARNING: System is UNSTABLE!")

except Exception as e:
    print(f"\nError computing LQR gains: {e}")
    exit(1)

print("\n" + "=" * 60)
