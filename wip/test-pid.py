import matplotlib.pyplot as plt
import numpy as np

from pid import PIDRegulator

# Define the PID parameters
Kp = 1.0
Ki = 0.1
Kd = 0.01
setpoint = 50.0
dt = 0.1  # Time step

# Initialize the PID regulator
pid = PIDRegulator(Kp, Ki, Kd, setpoint, dt)

# Simulate the system
simulation_time = 100  # Total simulation time
time = np.arange(0, simulation_time, dt)
current_value = 0.0  # Initial value of the process variable

# Lists to store the results
outputs = []
values = []

for t in time:
    # Update the PID regulator
    output = pid.update(current_value)

    # Simulate the system (first-order system with some delay)
    current_value += (output - current_value) * dt

    # Store the results
    outputs.append(output)
    values.append(current_value)

# Plot the results with dark theme
plt.style.use("dark_background")
plt.figure(figsize=(10, 6))

# Plot the process variable
plt.plot(time, values, label="Process Variable", color="#d3869b")

# Plot the setpoint
plt.plot(time, [setpoint] * len(time), "--", label="Setpoint", color="#83a598")

# Plot the control output
plt.plot(time, outputs, label="Control Output", color="#fabd2f")

# Customize the plot
plt.xlabel("Time (s)", color="white")
plt.ylabel("Value", color="white")
plt.title("PID Control Simulation", color="white")
plt.legend(facecolor="#1a1a1a", labelcolor="white")
plt.grid(True, color="gray", linestyle="--", linewidth=0.5)

# Set the background color of the plot area
ax = plt.gca()
ax.set_facecolor("#1a1a1a")

# Set the background color of the figure
fig = plt.gcf()
fig.patch.set_facecolor("#1a1a1a")

# Show the plot
plt.show()
