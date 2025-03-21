import numpy as np
import control as ctrl

# Define the Acrobot system parameters
m1 = 1.0  # mass of link 1 (kg)
m2 = 1.0  # mass of link 2 (kg)
l1 = 1.0  # length of link 1 (m)
l2 = 1.0  # length of link 2 (m)
lc1 = 0.5  # distance from joint to center of mass of link 1 (m)
lc2 = 0.5  # distance from joint to center of mass of link 2 (m)
I1 = 1.0  # moment of inertia of link 1 about its center of mass (kg*m^2)
I2 = 1.0  # moment of inertia of link 2 about its center of mass (kg*m^2)
g = 9.81  # acceleration due to gravity (m/s^2)

# Define the state-space representation of the Acrobot system
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, (m1*g*lc1 + m2*g*l1) / (m1*lc1**2 + m2*l1**2 + I1), 0, 0],
              [0, m2*g*lc2 / (m2*lc2**2 + I2), 0, 0]])

B = np.array([[0],
              [0],
              [1 / (m1*lc1**2 + m2*l1**2 + I1)],
              [1 / (m2*lc2**2 + I2)]])

C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

D = np.array([[0],
              [0]])

# Define the weighting matrices Q and R
Q = np.eye(4)  # State cost matrix
R = np.eye(1)  # Control cost matrix

# Solve the continuous-time algebraic Riccati equation (CARE) to obtain the optimal feedback gain matrix K
K, _, _ = ctrl.lqr(A, B, Q, R)

# Print the feedback gain matrix
print("Feedback gain matrix K:")
print(K)

# Simulate the system with the LQR controller
sys = ctrl.ss(A, B, C, D)
sys_with_controller = ctrl.ss(A - np.dot(B, K), B, C, D)

# Simulate step response of the closed-loop system
t = np.linspace(0, 10, 1000)
t, y = ctrl.step_response(sys_with_controller, T=t)

# Plot the results
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 6))
plt.plot(t, y[0].flatten(), label='theta1')
plt.plot(t, y[1].flatten(), label='theta2')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Acrobot Stabilization with LQR Controller')
plt.legend()
plt.grid(True)
plt.show()
