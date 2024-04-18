import numpy as np

# Define URDF parameters
l1, l2, l3, l4, l5, l6, l7 = 0.13, 0.063, 0.1900, 0.064, 0.2506, 0.075, 0.085

# Define random joint angles or set them to zero
delta1, delta2, delta3, delta4, delta5, delta6 = 0, 0, 0, 0, 0, 0



def rotx(a, b):
    return np.array([[1, 0, 0, 0],
                     [0, a, -b, 0],
                     [0, b, a, 0],
                     [0, 0, 0, 1]])

def roty(a, b):
    return np.array([[a, 0, b, 0],
                     [0, 1, 0, 0],
                     [-b, 0, a, 0],
                     [0, 0, 0, 1]])

def rotz(a, b):
    return np.array([[a, -b, 0, 0],
                     [b, a, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def transx(a):
    return np.array([[1, 0, 0, a],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def transy(a):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, a],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def transz(a):
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, a],
                     [0, 0, 0, 1]])












# Compute trigonometric functions
c1, s1 = np.cos(delta1), np.sin(delta1)
c2, s2 = np.cos(delta2), np.sin(delta2)
c3, s3 = np.cos(delta3), np.sin(delta3)
c4, s4 = np.cos(delta4), np.sin(delta4)
c5, s5 = np.cos(delta5), np.sin(delta5)
c6, s6 = np.cos(delta6), np.sin(delta6)

# Define transformation matrices
T12 = np.dot(rotz(c1, s1), transz(l1 + l2))
T23 = np.dot(roty(c2, s2), transz(l3))
T34 = np.dot(roty(c3, s3), transz(l4))
T45 = np.dot(transx(l5), rotx(c4, s4))
T56 = np.dot(roty(c5, s5), transx(l6))
T67 = np.dot(rotx(c6, s6), transx(l7))

T06test = np.dot(np.dot(np.dot(np.dot(np.dot(T12, T23), T34), T45), T56), T67).dot(roty(0, 1))

xEF = 0.042  # X position
yEF = 0 # Y position
zEF = 0.046 # Z position
# Extract end effector position and rotation
#xEF, yEF, zEF = T06test[:3, 3]
print(T06test)
print(xEF, yEF, zEF)
r13, r23, r33 = T06test[:3, 2]

# Define position c
d6 = l6 + l7
xc = xEF - d6 * r13
yc = yEF - d6 * r23
zc = zEF - d6 * r33

# Compute theta1, theta2, and theta3 using geometric solution
s = zc - (l1 + l2)
r = np.sqrt(xc ** 2 + yc ** 2)
a2 = l3
a3 = np.sqrt(l4 ** 2 + l5 ** 2)
c3 = (s ** 2 + r ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)

# 2 scenarios for theta1
theta1 = 180 + np.arctan2(yc, xc) * 180 / np.pi
theta11 = np.arctan2(yc, xc) * 180 / np.pi

# Handle scenario 1
the3 = (np.arccos(c3) * 180 / np.pi)
print(the3)
if the3 >= 0:
    theta3 = -90 - the3 + 14.32
else:
    theta3 = -90 + the3 + 14.32

tb = np.arctan2(s, r)
ta = np.arctan2(a3 * np.sin(the3 * np.pi / 180), a2 + a3 * np.cos(the3 * np.pi / 180))
the2 = (tb - ta) * 180 / np.pi
print(the2)
if the2 > 0:
    theta2 = 180 - 90 - abs(the2)
else:
    theta2 = 90 + abs(the2)

if theta1 > 180:
    theta1 = theta1 - 360

# Handle other scenarios similarly...

# Define T03 and extract rotational matrix
T03 = np.dot(np.dot(rotz(np.cos(theta1 * np.pi / 180), np.sin(theta1 * np.pi / 180)), 
                     roty(np.cos(theta2 * np.pi / 180), np.sin(theta2 * np.pi / 180))), 
             roty(np.cos(theta3 * np.pi / 180), np.sin(theta3 * np.pi / 180)))
R03 = T03[:3, :3]

# Compute R36 and other rotations similarly...

# Print or use thetas table
print("Theta Values:")
print("-------------")
print("Theta1:", theta1)
print("Theta2:", theta2)
print("Theta3:", theta3)
# Print or use other theta values similarly...













# Define theta values obtained previously









# Define T03
T03 = np.dot(np.dot(rotz(np.cos(theta1 * np.pi / 180), np.sin(theta1 * np.pi / 180)), 
                    roty(np.cos(theta2 * np.pi / 180), np.sin(theta2 * np.pi / 180))), 
             roty(np.cos(theta3 * np.pi / 180), np.sin(theta3 * np.pi / 180)))
T031 = np.dot(np.dot(rotz(np.cos(theta11 * np.pi / 180), np.sin(theta11 * np.pi / 180)), 
                     roty(np.cos(theta2 * np.pi / 180), np.sin(theta2 * np.pi / 180))), 
              roty(np.cos(theta3 * np.pi / 180), np.sin(theta3 * np.pi / 180)))
# Define and compute other T0x matrices similarly...

# Extract rotational matrix
R03 = T03[:3, :3]
R031 = T031[:3, :3]
# Extract rotational matrices for other cases similarly...

R06 = T06test[:3, :3]

# Obtain rotational matrix for spherical wrist
R36 = np.dot(np.linalg.inv(R03), R06)
R361 = np.dot(np.linalg.inv(R031), R06)
# Obtain rotational matrices for other cases similarly...

# Find theta 4,5,6 with the general rotational matrix
# R36 = [          -c6*s5,            s5*s6,     c5]
#       [c4*s6 + c5*c6*s4, c4*c6 - c5*s4*s6,  s4*s5]
#       [s4*s6 - c4*c5*c6, c6*s4 + c4*c5*s6, -c4*s5]

if np.abs(R36[2, 2]) != 1:
    c5 = R36[0, 2]
    s5 = np.sqrt(1 - c5 ** 2)
    s51 = -np.sqrt(1 - c5 ** 2)
    # Two cases of theta5
    theta5 = np.arctan2(s5, c5) * 180 / np.pi
    theta51 = np.arctan2(s51, c5) * 180 / np.pi

# Handle theta5 >= 0
if s5 >= 0:
    theta4 = np.arctan2(R36[1, 2], -R36[2, 2]) * 180 / np.pi
    theta6 = np.arctan2(R36[0, 1], -R36[0, 0]) * 180 / np.pi
elif s5 < 0:
    theta4 = np.arctan2(-R36[1, 2], R36[2, 2]) * 180 / np.pi
    theta6 = np.arctan2(-R36[0, 1], R36[0, 0]) * 180 / np.pi

# Handle theta51 >= 0
if s5 > 0:
    theta4= np.arctan2(R36[1, 2], -R36[2, 2]) * 180 / np.pi
    theta6= np.arctan2(R36[0, 1], -R36[0, 0]) * 180 / np.pi
elif s5 < 0:
    theta4 = np.arctan2(-R36[1, 2], R36[2, 2]) * 180 / np.pi
    theta6 = np.arctan2(-R36[0, 1], R36[0, 0]) * 180 / np.pi

# Print or use theta values as needed
print("Theta 4: ", theta4)
print("Theta 5: ", theta5)
print("Theta 6: ", theta6)
