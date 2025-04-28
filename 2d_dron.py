import numpy as np
import matplotlib.pyplot as plt
import math

# Simulation constants
m = 5               # Dron mass (kg)
g = 9.8             # Erth gravity (m/s²)
Fg = m*g
d = 0.2             # Distance motors-centre (m)
I = (1/3)*m*(d**2)  # Inertia (kg*m²)
max_time = 120 # Simulation time (s)
dt = 0.01            # Time step
n_steps = int(max_time / dt)
time_steps = np.linspace(0, max_time, n_steps)

# Controller gains
Kp_theta = 0.01     
Kp_y = 0.8
Ki_y = 0.0001
Kd_y = 3

# Reference values
x_ref = 0.0
y_ref = 0.0
theta_ref = 0.0

# Initial conditions
starting_x=0.0
starting_y=0.0
starting_theta=0.0
starting_vx=0.0
starting_vy=0.0
starting_omega=0.0

def simulate_system():
    x = starting_x
    y = starting_y
    theta = starting_theta
    vx = starting_vx
    vy = starting_vy
    omega = starting_omega

    x_positions = []
    y_positions = []
    vx_instances = []
    vy_instances = []

    integral_error_y = 0
    derivative_error_y = 0
    previous_error_y = 0

    for _ in range(n_steps):

        error_x = x_ref - x
        error_y = y_ref - y
        integral_error_y += error_y * dt
        derivative_error_y = (error_y - previous_error_y) / dt

        theta_error = theta_ref - theta

        torque = Kp_theta * theta_error
        delta_F = torque * I / d    # Force imbalance between motors

        Ftotal = Fg + (Kp_y * error_y) + (Ki_y * integral_error_y) + (Kd_y * derivative_error_y)

        Fl = (Ftotal - delta_F) / 2
        Fl = (Ftotal + delta_F) / 2

        Fl = np.clip(Fl, 0, 100)
        Fr = np.clip(Fl, 0, 100)

        # Vertical movement
        ay = ((Fl + Fr)*math.cos(theta) - Fg)/m
        vy += ay * dt

        # Horizontal movement
        ax = ((Fl + Fr)*math.sin(theta))/m
        vx += ax * dt

        # Angular movement
        a_ang = ((Fl - Fr)*d)/I
        omega += a_ang * dt

        x += vx * dt
        y += vy * dt
        
        theta += omega * dt

        previous_error_y = error_y

        x_positions.append(x)
        y_positions.append(y)
        vx_instances.append(vx)
        vy_instances.append(vy)

    return x_positions, y_positions, vx_instances, vy_instances


x_positions, y_positions, _, _ = simulate_system()

# Plotting
fig, axs = plt.subplots(3, 1, figsize=(20, 12), sharex=True)

axs[0].plot(time_steps, x_positions, label='X Position', color='blue')
axs[0].set_title('Dron Horizontal Position vs Time')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('X Position (m)')
axs[0].grid()
axs[0].legend()

axs[1].plot(time_steps, y_positions, label='Y Position', color='green')
axs[1].set_title('Dron Vertical Position vs Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Y Position (m)')
axs[1].grid()
axs[1].legend()

axs[2].plot(x_positions, y_positions, label='Dron Path', color='red')
axs[2].set_title('Dron Path')
axs[2].set_xlabel('X Position (m)')
axs[2].set_ylabel('Y Position (m)')
axs[2].grid()
axs[2].legend()

fig.tight_layout()
plt.show()
