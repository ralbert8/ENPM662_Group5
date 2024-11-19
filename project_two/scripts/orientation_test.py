#!/usr/bin/env python3

import sympy as s
import numpy as n
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import os
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R, Slerp

package_share_dir = get_package_share_directory('project_two')



#################
# DH Parameters #
#################

# Create Symbols for DH Parameters
theta1, theta2, theta3, theta4, theta5, theta6 = s.symbols('theta1 theta2 theta3 theta4 theta5 theta6')
a2, a3 = s.symbols('a2 a3')
d1, d4, d5, d6 = s.symbols('d1 d4 d5 d6')

# Define DH Table
dh_parameters = [
    [theta1,          d1, 0,  -s.pi/2],
    [theta2 - s.pi/2, 0,  a2, 0],
    [theta3,          0,  a3, 0],
    [theta4 + s.pi/2, d4, 0,  s.pi/2],
    [theta5,          d5, 0,  -s.pi/2],
    [theta6,          d6, 0,  0]
]

# Convert DH_i Parameters to A_i Matrix
def dh_to_transform(theta, d, a, alpha):
    A = s.Matrix([
        [s.cos(theta), -s.sin(theta) * s.cos(alpha), s.sin(theta) * s.sin(alpha),  a * s.cos(theta)],
        [s.sin(theta), s.cos(theta) * s.cos(alpha),  -s.cos(theta) * s.sin(alpha), a * s.sin(theta)],
        [0,            s.sin(alpha),                 s.cos(alpha),                 d],
        [0,            0,                            0,                            1]
    ])
    return A

# Initialize Storage for Matrices
A_matrices = []
T_base = []


###################################
# Successive Link Transformations #
###################################

# Compute A_i Matrices
for params in dh_parameters:
    A_i = dh_to_transform(*params)
    A_matrices.append(A_i)



########################
# Final Transformation #
########################

# Final Transformation for Each Link
T_prev = s.eye(4)
for A in A_matrices:
    T_prev = s.trigsimp(T_prev * A)
    T_base.append(T_prev)



######################
# Forward Kinematics #
######################

# Insert DH Parameters
parameter_substitutions = {
    a2: 0.425,
    a3: 0.39225,
    d1: 0.1625,
    d4: 0.09465,
    d5: 0.09465,
    d6: 0.0815
}

# Define Forward Kinematics of Robot
def forward_kinematics(t1, t2, t3, t4, t5, t6, display=False):
    T_Full_Numeric = T_base[-1].subs(parameter_substitutions)
    theta_substitutions = {
        theta1: t1,
        theta2: t2,
        theta3: t3,
        theta4: t4,
        theta5: t5,
        theta6: t6
    }
    T_Theta_Subs = T_Full_Numeric.subs(theta_substitutions)

    if display:
        s.pprint(T_Theta_Subs[:3, 3])
        s.pprint(T_Theta_Subs[:3, :3])

    return T_Theta_Subs[:3, 3], T_Theta_Subs[:3, :3]  # Position and Orientation



###################
# Jacobian Matrix #
###################

# Initialize O and Z Vector Storage
O_vectors = [s.Matrix([0, 0, 0])]
Z_vectors = [s.Matrix([0, 0, 1])]

# Gather O_i and Z_i for each Transformation
for T in T_base:
    O_vectors.append(T[:3, 3])
    Z_vectors.append(T[:3, 2])

# Initialize J Vector Storage
J_vectors = []

# Gather O_n
O_n = O_vectors[-1]

# Compute J_i
for i in range(len(Z_vectors) - 1):
    O_prev = O_vectors[i]
    Z_prev = Z_vectors[i]
    cross = Z_prev.cross(O_n - O_prev)
    J_i = s.Matrix.vstack(cross, Z_prev)
    J_vectors.append(s.trigsimp(J_i))

# Assemble Parametric Jacobian
Jacobian_Parametric = s.Matrix.hstack(*J_vectors)

# Subsitiute DH Parameters to Gather Jacobian as a Function of Thetas
Jacobian_Thetas = Jacobian_Parametric.subs(parameter_substitutions)



###################
# Path Definition #
###################

# Define Function to Return the Position at Time t for the Path
def position_vector_function(t):
    position = s.Matrix([x_path(t), y_path(t), z_path(t)])
    return position



######################
# Inverse Kinematics #
######################

all_joint_angles = []
# Define the Function for Inverse Kinematics using Numerical Integration
def inverse_kinematics(current_joints, move_time, target_orientation, dt=0.01):
    current_position, current_orientation_matrix = forward_kinematics(*current_joints)

    current_orientation = R.from_matrix(n.array(current_orientation_matrix, dtype=float))
    target_orientation = R.from_matrix(n.array(target_orientation, dtype=float))

    key_times = [0, move_time]
    key_rotations = [current_orientation, target_orientation]
    slerp = Slerp(key_times, R.concatenate(key_rotations))

    global all_joint_angles  # Access the global joint angle storage

    time_array = n.arange(0, move_time + dt / 2, dt)

    for t in time_array:
        # Determine the Next Interpolated Point along Path
        interpolated_position = position_vector_function(t + dt)

        interpolated_rotation = slerp([t])[0]
        interpolated_rotation_matrix = interpolated_rotation.as_matrix()

        orientation_error_matrix = target_orientation @ interpolated_rotation_matrix.T
        rotation_axis_angle = R.from_matrix(orientation_error_matrix).as_rotvec()
        ang_velocity = rotation_axis_angle / dt

        ang_velocity_sympy = s.Matrix(ang_velocity)

        # Set Current Angles for Substitution
        theta_substitutions = {
            theta1: current_joints[0],
            theta2: current_joints[1],
            theta3: current_joints[2],
            theta4: current_joints[3],
            theta5: current_joints[4],
            theta6: current_joints[5]
        }

        # Substitute Current Angles into Jacobian
        Jacobian = Jacobian_Thetas.subs(theta_substitutions)

        # Determine Distance to Next Interpolated Point
        position_error = interpolated_position - current_position

        # Determine Linear Velocity of End Effector to Reach Next Interpolated Point over Time Step
        lin_velocity = position_error / dt

        # Assemble Velocity Vector
        velocity = s.Matrix.vstack(lin_velocity, ang_velocity_sympy)

        # Compute Joint Velocities
        joint_velocity = Jacobian.pinv() * velocity

        # Update Joint Angles Through Numeric Integration
        current_joints += joint_velocity * dt

        # Store Current Joint Angles in Global List
        all_joint_angles.append([float(joint) for joint in current_joints])

        # Update Current Position and Orientation for Next Iteration
        current_position, current_orientation = forward_kinematics(*current_joints, display=True)

        # Store Position for Plotting
        x_plot.append(float(current_position[0]))
        y_plot.append(float(current_position[1]))
        z_plot.append(float(current_position[2]))

    return current_joints



#######################
# Initial Robot State #
#######################

# Set Initial Joint States and Gather Initial Position and Orientation
current_joints = s.Matrix([0, n.pi/4, -n.pi/2, n.pi/4, -n.pi/2, 0])
current_position, current_orientation = forward_kinematics(*current_joints)

# Read First Row of CSV to Determine Drawing Start
input_file = os.path.join(package_share_dir, 'csv', 'contours.csv')
with open(input_file, mode="r") as file:
    reader = csv.reader(file)
    next(reader)
    first_row = next(reader)
    draw_start = [float(value) for value in first_row]



###########################
# Travel to Drawing Start #
###########################

target_position = draw_start
target_orientation = s.Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

# Total Time for the Movement
move_time = 1

# Define Path From Starting Point to Beginning of Drawing as a Vector Function of Time
x_path = lambda t: (((target_position[0] - current_position[0]) / move_time) * t) + current_position[0]
y_path = lambda t: (((target_position[1] - current_position[1]) / move_time) * t) + current_position[1]
z_path = lambda t: (((target_position[2] - current_position[2]) / move_time) * t) + current_position[2]

x_plot, y_plot, z_plot = [], [], []

current_joints = inverse_kinematics(current_joints, move_time, target_orientation)



##############################
# Output Joint Angles to CSV #
##############################

output_file = os.path.join(package_share_dir, 'csv', 'joint_angles.csv')
with open(output_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Theta1", "Theta2", "Theta3", "Theta4", "Theta5", "Theta6"])  # Header
    writer.writerows(all_joint_angles)  # Write all joint angle data



##########################
# Plot End Effector Path #
##########################

# Plot Path in 3D Space
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_plot, y_plot, z_plot, label="End Effector Path")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.legend()
plt.show()