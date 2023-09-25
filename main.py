# Dynamic function
def dynamic_output(x_prev, plant_output, a, dt):
    return x_prev + dt * a * (x_prev - plant_output)

# PID controller class
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# Set desired X, Y, Z values
desired_X = 15
desired_Y = -10
desired_Z = 5

# PID controller parameters
Kp1 = 0.8
Ki1 = 0
Kd1 = 0.1

Kp2 = 2
Ki2 = 0.1
Kd2 = 0.1

Kp3 = 5
Ki3 = 0.2
Kd3 = 1

dt = 0.1  # time step

# Create PID controllers for X, Y, and Z
pid_X = PID(Kp1, Ki1, Kd1, desired_X, dt)
pid_Y = PID(Kp2, Ki2, Kd2, desired_Y, dt)
pid_Z = PID(Kp3, Ki3, Kd3, desired_Z, dt)

# Control loop
max_iterations = 430
errors_X, errors_Y, errors_Z = [], [], []
positions_X, positions_Y, positions_Z = [], [], []  # lists to store positions

input_values = np.array([0, 0, 70, 10, -50], dtype=np.float64)  # Initial input values

for _ in range(max_iterations):
    # Get the current X, Y, Z values from the plant
    input_data = input_values.reshape(1, -1)
    input_data = input_scaler.transform(input_data)
    plant_output = output_scaler.inverse_transform(plant.predict(input_data))[0]

    # If first iteration, initialize dynamic output as plant output
    if _ == 0:
        dynamic_output_X, dynamic_output_Y, dynamic_output_Z = plant_output
    else:
        # Generate dynamic output
        dynamic_output_X = dynamic_output(dynamic_output_X, plant_output[0], a=-2, dt=dt)
        dynamic_output_Y = dynamic_output(dynamic_output_Y, plant_output[1], a=-2, dt=dt)
        dynamic_output_Z = dynamic_output(dynamic_output_Z, plant_output[2], a=-2, dt=dt)

    # Update PID controllers
    control_X = pid_X.update(dynamic_output_X)
    control_Y = pid_Y.update(dynamic_output_Y)
    control_Z = pid_Z.update(dynamic_output_Z)

    # Update input values based on control signals
    input_values[2] += control_X
    input_values[3] += control_Y
    input_values[4] += control_Z

    # Store positions for plotting
    positions_X.append(dynamic_output_X)
    positions_Y.append(dynamic_output_Y)
    positions_Z.append(dynamic_output_Z)

    # Log errors for plotting
    errors_X.append(pid_X.previous_error)
    errors_Y.append(pid_Y.previous_error)
    errors_Z.append(pid_Z.previous_error)

# Plot errors
plt.figure(figsize=(12,8))
plt.subplot(2, 1, 1)
plt.plot(errors_X, label='Error X')
plt.plot(errors_Y, label='Error Y')
plt.plot(errors_Z, label='Error Z')
plt.xlabel('Iteration')
plt.ylabel('Error')
plt.legend()
plt.title('PID Controller Errors')
plt.grid(True)

# Plot positions
plt.subplot(2, 1, 2)
plt.plot(positions_X, label='Position X')
plt.plot(positions_Y, label='Position Y')
plt.plot(positions_Z, label='Position Z')
plt.xlabel('Iteration')
plt.ylabel('Position')
plt.legend()
plt.title('Positions of X, Y, Z')
plt.grid(True)

plt.tight_layout()
plt.show()
