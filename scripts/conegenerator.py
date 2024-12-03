import numpy as np
import matplotlib.pyplot as plt
# The arc is a semi-circle, from pi (180 degrees) to 0
theta = np.linspace(np.pi, 0, int(2 * np.pi * 3))  # Assuming there's a point every 1.5m in the outer arc with radius 6m
# Define the radius of outer and inner semi-circles

curve_radius = 6
track_width = 5

r_outer = curve_radius + track_width/2  # radius of outer semi-circle
r_inner = curve_radius - track_width/2  # radius of inner semi-circle
# Compute x, y coordinates for points along the outer and inner semi-circles


y_outer_arc = r_outer * np.cos(theta) + r_outer - (r_outer-r_inner)/2
x_outer_arc = r_outer * np.sin(theta) + 20
y_inner_arc = r_inner * np.cos(theta) + r_outer - (r_outer-r_inner)/2 # Offset by r_outer to align with outer arc
x_inner_arc = r_inner * np.sin(theta) + 20

# Compute the straight line sections (assuming they are 10m long)
x_straight = np.linspace(2, 19, int(15/1))  # Points every 2m for the straight line
x_straight_exit = np.linspace(2, 19, int(15/1))  # Exit straight line
y_outer_straight = np.full_like(x_straight, - (r_outer-r_inner)/2)  # Outer straight line is at x = 0
y_inner_straight = np.full_like(x_straight, + (r_outer-r_inner)/2)  # Inner straight line is at x = 3
y_outer_straight_exit = np.full_like(x_straight_exit, 2*r_outer - (r_outer-r_inner)/2)  # Outer exit straight line is at x = 2*r_outer
y_inner_straight_exit = np.full_like(x_straight_exit, 2*r_outer - (r_outer-r_inner)/2 - (r_outer-r_inner))  # Inner exit straight line is at x = 2*r_outer - 3

x_orange = [1,1,1,1]
y_orange = [track_width/2, - track_width/2, 2*curve_radius + track_width/2, 2*curve_radius - track_width/2]

# Concatenate straight line, arc sections and exit straight line
x_outer = np.concatenate((x_straight, x_outer_arc, x_straight_exit))
y_outer = np.concatenate((y_outer_straight, y_outer_arc, y_outer_straight_exit))
x_inner = np.concatenate((x_straight, x_inner_arc, x_straight_exit))
y_inner = np.concatenate((y_inner_straight, y_inner_arc, y_inner_straight_exit))

# Create the plot
plt.figure(figsize=(10, 10))
plt.plot(x_outer, y_outer, 'o', label='Outer Path')
plt.plot(x_inner, y_inner, 'o', label='Inner Path')
plt.plot(x_orange, y_orange, 'o', label='Orange Cone')

s = ""
for i in range(len(x_inner)):
    s += "yellow,%f,%f,0,0,0,0\n" % (x_inner[i],y_inner[i])

for i in range(len(x_outer)):
    s += "blue,%f,%f,0,0,0,0\n" % (x_outer[i],y_outer[i])

for i in range(len(x_orange)):
    s += "big_orange,%f,%f,0,0,0,0\n" % (x_orange[i],y_orange[i])


f = open("map.csv", 'w')
f.write(s)
f.close()

print(s)

plt.legend()
plt.title('U-turn Track with Entry and Exit Straight Sections')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.grid(True)
plt.axis('equal')  # Ensure the x and y axis scales are equal
plt.show()