import pandas as pd
import numpy as np

import matplotlib.pyplot as plt

df = pd.read_csv('sensorData.csv')

X = df['step_dist'].to_numpy()
Y = df['reading'].to_numpy()


# Perform the linear regression (degree=1 for a linear line)
m, b = np.polyfit(X, Y, 1)

print(f"Slope (m): {m}")
print(f"Y-intercept (b): {b}")

ycalc = X * m + b

plt.plot(X,Y)
plt.plot(X, ycalc)

plt.legend(["Raw Data", "Linear Regression"])

plt.title("Sensor Calibration")

plt.xlabel("Step Reading")

plt.ylabel("Sensor Reading (mm)")

plt.show()