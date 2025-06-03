from scara import Scara  # Assuming the Scara class is available in scara.py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Initialize the Scara object with two linkage lengths
scara_robot = Scara((50, 50))  # Example linkage lengths

# Create the figure for 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Loop through the grid of x and y values
for x in range(-100, 101, 5):
    for y in range(-100, 101, 5):
        try:
            # Get the angles for the current position
            angles = scara_robot.inverse((x, y))
            # Plot the point with Z up orientation and the angles as two Y values
            ax.scatter(x, angles[0], angles[1], c='b', marker='o')
        except Exception as e:
            # If the position is out of reach, we can either skip it or handle it differently
            print(f"Cannot reach ({x}, {y}): {e}")

# Set labels
ax.set_xlabel('X axis')
ax.set_ylabel('First Angle Y axis')
ax.set_zlabel('Second Angle Y axis')

# Show the plot
plt.show()
