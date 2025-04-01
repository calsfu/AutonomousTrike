# Note, not going to use trajectory planning since we cannot control velocity or acceleration.
# trajectory planning uses parametric splines, meaning velocity and accel is required

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Example GPS points (replace with your actual data)
gps_points = np.array([[0, 0], [1, 2], [3, 3], [5, 1]])

# 1. Path Representation (Cubic Spline)
spline = CubicSpline(gps_points[:, 0], gps_points[:, 1])

# Generate points along the spline for plotting
x_spline = np.linspace(gps_points[0, 0], gps_points[-1, 0], 100) #100 points between first and last x coordinate.
y_spline = spline(x_spline)

# Plot the GPS points and the spline
plt.figure(figsize=(8, 6))
plt.plot(gps_points[:, 0], gps_points[:, 1], 'o', label='GPS Points') # plot gps points as dots.
plt.plot(x_spline, y_spline, '-', label='Cubic Spline') #plot spline as line.
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('GPS Points and Cubic Spline')
plt.legend()
plt.grid(True)
plt.show()

#If you want to plot the derivatives:
dx = spline.derivative(1)(x_spline)
ddx = spline.derivative(2)(x_spline)

plt.figure(figsize=(8,6))
plt.plot(x_spline, dx, label = 'First derivative')
plt.plot(x_spline, ddx, label = 'Second derivative')
plt.xlabel('X coordinate')
plt.ylabel('Derivative value')
plt.title('Spline derivatives')
plt.grid(True)
plt.legend()
plt.show()