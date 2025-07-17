import random
import matplotlib.pyplot as plt
import math

kp = 3
ki = 0
kd = 1.5
setpoint = 0
integral = 0
prev_error = 0

SAMPLE_TIME = 10  # milliseconds
FAN_0_SPEED_PWM = 10000*0.05
FAN_MAX_SPEED_PWM = 10000*0.05*2
FAN_MID_SPEED_PWM = (FAN_0_SPEED_PWM + FAN_MAX_SPEED_PWM) / 2.0

# sensor_data = [int(70 * math.sin(x * math.pi / 10)) for x in range(20)] // sinc function
sensor_data = [0 if x < 5 else 50 for x in range(20)]

def cal(sensor_data):
  global kp, ki, kd, setpoint, integral, prev_error
  error = setpoint - sensor_data

  p_term = kp * error
  # print(f'P term: {p_term}')

  integral += error * SAMPLE_TIME
  i_term = ki * integral
  # print(f'I term: {i_term}')

  d_term = kd * (error - prev_error) / SAMPLE_TIME
  # print(f'D term: {d_term}')
  prev_error = error

  return p_term + i_term + d_term

pid_outputs = [cal(data) for data in sensor_data]

# print("Sensor Data:", sensor_data)
# print("PID Outputs:", pid_outputs)

# Create a new array to store the results, clamped to bounds
result_array = [
  max(FAN_0_SPEED_PWM, min(FAN_MID_SPEED_PWM + data, FAN_MAX_SPEED_PWM))
  for data in pid_outputs
]

# Create another array without bounding
unbounded_result_array = [
  FAN_MID_SPEED_PWM + data
  for data in pid_outputs
]

# Print the result arrays
# print("Result Array (bounded):", result_array)
# print("Result Array (unbounded):", unbounded_result_array)

# Plot both arrays in the same graph
plt.figure(figsize=(10, 6))
plt.plot(result_array, label="Result Array (bounded)", marker='s', color='g')
plt.plot(unbounded_result_array, label="Result Array (unbounded)", marker='d', color='b')
plt.title("Result Array Plot")
plt.xlabel("Index")
plt.ylabel("Value")
plt.ylim(500, 1000)  # Set y-axis limits
plt.legend()
plt.grid()
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(sensor_data, label="Sensor Data", marker='o')
plt.plot(pid_outputs, label="PID Output", marker='x')
plt.axhline(y=setpoint, color='r', linestyle='--', label="Setpoint")
plt.title("Sensor Data and PID Output")
plt.xlabel("Index")
plt.ylabel("Value")
plt.legend()
plt.grid()
plt.show()