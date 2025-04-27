import numpy as np
import matplotlib.pyplot as plt
import math

# Constants
m = 5               # Dron mass (kg)
g = 9.8             # Erth gravity (m/s²)
d = 0.2             # Distance motors-centre (m)
I = (1/3)*m*(d**2)  # Inertia (kg*m²)
max_time = 120      # Simulation time (s)

# Simulation parameters
x = 0.0
y = 5.0             # Starting height at 5m
theta = 0.0         # Angle to vertical axis
vx = 0.0
vy = 0.0
omega = 0.0         # Angular velocity
dt = 0.01            # Time step
n_steps = int(max_time / dt)
time_steps = np.linspace(0, max_time, n_steps)

# Forces
Fg = m*g            # Gravity force (N)
Fl = 24.5             # Left motor force (N)
Fr = Fl             # Right motor force (N)

x_positions = []
y_positions = []

for _ in range(n_steps):
    # Vertical movement
    ay = ((Fl + Fr)*math.cos(theta) - Fg)/m
    vy += ay * dt

    # Horizontal movement
    ax = ((Fl + Fr)*math.sin(theta))/m
    vx += ax * dt

    # Angular movement
    a_ang = (Fr - Fl)*d/I
    omega += a_ang * dt

    x += vx * dt
    y += vy * dt
    
    # Simulate ground collision
    if y < 0:
        y = 0
        vy = 0

    theta += omega * dt

    x_positions.append(x)
    y_positions.append(y)

# Plotting
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(time_steps, x_positions, label='X Position', color='blue')
plt.title('Dron Horizontal Path')
plt.xlabel('Time (s)')
plt.ylabel('X Position (m)')
plt.xlim(0, max_time)
plt.grid()
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_steps, y_positions, label='Y Position', color='green')
plt.title('Dron Vertical Path')
plt.xlabel('Time (s)')
plt.ylabel('Y Position (m)')
plt.xlim(0, max_time)
plt.grid()
plt.legend()

plt.show()

