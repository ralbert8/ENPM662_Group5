#!/usr/bin/env python3

import sympy as s
import numpy as n
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import os
from ament_index_python.packages import get_package_share_directory

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
    d1: 0.58365,
    d4: 0.1099,
    d5: 0.09065,
    d6: 0.2002
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
def inverse_kinematics(current_joints, move_time, dt=0.01):
    current_position, current_orientation = forward_kinematics(*current_joints)
    global all_joint_angles  # Access the global joint angle storage
    time_array = n.arange(0, move_time + dt / 2, dt)

    for t in time_array:
        # Determine the Next Interpolated Point along Path
        interpolated_position = position_vector_function(t + dt)

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

        # Determine Orientation Error
        orientation_error = target_orientation * current_orientation.inv() - s.eye(3)

        # Determine Angular Velocity of End Effector to Reach Target Orientation over Time Step
        ang_velocity = s.Matrix([orientation_error[2, 1], orientation_error[0, 2], orientation_error[1, 0]]) / dt

        # Assemble Velocity Vector
        velocity = s.Matrix.vstack(lin_velocity, ang_velocity)

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
target_orientation = current_orientation

# Total Time for the Movement
move_time = 1

# Define Path From Starting Point to Beginning of Drawing as a Vector Function of Time
x_path = lambda t: (((target_position[0] - current_position[0]) / move_time) * t) + current_position[0]
y_path = lambda t: (((target_position[1] - current_position[1]) / move_time) * t) + current_position[1]
z_path = lambda t: (((target_position[2] - current_position[2]) / move_time) * t) + current_position[2]

x_plot, y_plot, z_plot = [], [], []

current_joints = inverse_kinematics(current_joints, move_time)



#######################
# Travel Drawing Path #
#######################

# Create Storage for Drawing Points
drawing_target = []

# Read From CSV
with open(input_file, mode="r") as file:
    reader = csv.reader(file)
    next(reader)
    next(reader)
    rows = list(reader)

for row in rows:
    move_time = 0.02 

    current_position, current_orientation = forward_kinematics(*current_joints)

    target_position = [float(value) for value in row]
    
    x_path = lambda t: (((target_position[0] - current_position[0]) / move_time) * t) + current_position[0]
    y_path = lambda t: (((target_position[1] - current_position[1]) / move_time) * t) + current_position[1]
    z_path = lambda t: (((target_position[2] - current_position[2]) / move_time) * t) + current_position[2]

    current_joints = inverse_kinematics(current_joints, move_time)



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
ax.plot(x_plot, y_plot, z_plot, label="3D Commanded End Effector Path")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.legend()
plt.title("3D Commanded End Effector Path")
# plt.show()

plotname = "3D_Commanded_EndEffector_Path.png"
plt.savefig(plotname)
plotpath = os.path.join(os.getcwd(),plotname)
print(f"\nFinished generating end effector planned trajectory to sketch the scene! See trajectory plot: {plotpath}")