import numpy as np
import matplotlib.pyplot as plt

k = 0.015            # Heat transfer coef
heater_efficiency = 30.0  # Heater efficiency
thermal_mass = 150.0  # Thermal mass of the oven

learning_rate = 0.1

T_init = 25         # Initial temperature
T_amb = 20          # Ambient temperature
T_ref = 200         # Desired temperature

# Default gains
# Kp = 100.0            # Proportional gain
# Ki = 0.04             # Integral gain
# Kd = 5.0              # Derivative gain

h = 0.1               # Time step
t_max = 5000          # Max time simulation (seconds)

T = T_init
time_steps = np.arange(0, t_max, h)
temperatures = []
errors = []

def simulate_system(Kp, Ki, Kd, h, t_max):
    T = T_init
    time_steps = np.arange(0, t_max, h)
    temperatures = []
    errors = []

    integral_error = 0
    derivative_error = 0
    previous_error = 0

    for _ in time_steps:
        error = T_ref - T
        integral_error += error * h
        derivative_error = (error - previous_error) / h

        P = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
        P = np.clip(P, 0, 100)

        dT_dt = ( -k * (T - T_amb) + heater_efficiency * (P / 100.0) ) / thermal_mass
        T = T + dT_dt * h

        temperatures.append(T)
        errors.append(error)
        previous_error = error

    return temperatures, errors

def cost_function(Kp, Ki, Kd, h, t_max):
    _ , errors = simulate_system(Kp, Ki, Kd, h, t_max)
    return np.mean(np.array(errors) ** 2)   # Mean Squared Error (MSE)

def gradient_descent(initial_Kp, initial_Ki, initial_Kd, max_iterations=100):
    Kp = initial_Kp
    Ki = initial_Ki
    Kd = initial_Kd
    history = [] 

    delta = 1e-1
    min_Kp, min_Ki, min_Kd = 0, 0, 0
    max_Kp, max_Ki, max_Kd = 500, 10, 50

    for i in range(max_iterations):
        base_cost = cost_function(Kp, Ki, Kd, h, t_max)

        grad_Kp = (cost_function(Kp + delta, Ki, Kd, h, t_max) - base_cost) / delta
        grad_Ki = (cost_function(Kp, Ki + delta, Kd, h, t_max) - base_cost) / delta
        grad_Kd = (cost_function(Kp, Ki, Kd + delta, h, t_max) - base_cost) / delta

        Kp -= learning_rate * grad_Kp
        Ki -= learning_rate * grad_Ki
        Kd -= learning_rate * grad_Kd

        # Clipping the values to ensure they are within the defined limits
        Kp = np.clip(Kp, min_Kp, max_Kp)
        Ki = np.clip(Ki, min_Ki, max_Ki)
        Kd = np.clip(Kd, min_Kd, max_Kd)

        history.append((Kp, Ki, Kd, base_cost))

        if i % 10 == 0:
            print(f"Iteración {i}: Kp = {Kp:.4f}, Ki = {Ki:.4f}, Kd = {Kd:.4f}, Costo = {base_cost:.4f}")

    return Kp, Ki, Kd, history

initial_Kp = 0.0
initial_Ki = 0.0
initial_Kd = 0.0
optimal_Kp, optimal_Ki, optimal_Kd, history = gradient_descent(initial_Kp, initial_Ki, initial_Kd)

print(f"Optimal PID gains: Kp = {optimal_Kp:.4f}, Ki = {optimal_Ki:.4f}, Kd = {optimal_Kd:.4f}")

temperatures_optimal = simulate_system(optimal_Kp, optimal_Ki, optimal_Kd, h, t_max)[0]

plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)

plt.plot(time_steps, temperatures_optimal, label=f'Oven Temperature (Kp={optimal_Kp:.2f}, Ki={optimal_Ki:.4f}, Kd={optimal_Kd:.2f})', color='red')
plt.axhline(y=T_ref, color='blue', linestyle='--', label='Desired Temperature')
plt.xlabel('Time (s)')
plt.xlim(0, 2000)
plt.ylabel('Temperature (°C)')
plt.title('Oven Temperature Control with Optimized PID Gains')
plt.legend()

plt.tight_layout()
plt.show()
