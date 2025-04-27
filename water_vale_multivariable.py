import numpy as np
import matplotlib.pyplot as plt

k = 0.05  # Leak constant

h_ref = 1000.0  # Desired water level (m)
h_init = 0.0  # Initial water level (m)
max_t = 1000  # Max time simulation (seconds)
dt = 0.1  # Time step

n_steps = int(max_t / dt)
time_steps = np.linspace(0, max_t, n_steps)

# Constant gains
Kp = 0.01
Ki = 0.001
Kd = 0.01

Q_in_big_max = 50
Q_in_small_max = 10

error = 0
integral_error = 0
derivative_error = 0
previous_error = 0
height = h_init

heights = []

for _ in range(n_steps):
    error = h_ref - height
    integral_error += error * dt
    derivative_error = (error - previous_error) / dt
  
    control_signal = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)

    if abs(error) > (h_ref * 0.1):
        Q_in_big = np.clip(control_signal, 0, Q_in_big_max)
        Q_in_small = 0 # Close the small valve
    else:
        Q_in_big = 0 # Close the big valve
        Q_in_small = np.clip(control_signal, 0, Q_in_small_max)

    Q_in = Q_in_big + Q_in_small

    dh_dt = Q_in - k * height
    height += dh_dt * dt

    heights.append(height)
    previous_error = error

# Plotting
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time_steps, heights, label='Water Level (m)')
plt.axhline(h_ref, color='r', linestyle='--', label='Desired Water Level (m)')
plt.title('Water Level Control with Two Valves')
plt.xlabel('Time (s)')
plt.xlim(0, max_t)
plt.ylabel('Water Level (m)')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

