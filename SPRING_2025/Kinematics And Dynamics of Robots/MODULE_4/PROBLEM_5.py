import numpy as np
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Rotation matrices
def rotation_x(angle):
    rad = np.radians(angle)
    return np.array([
        [1, 0, 0],
        [0, np.cos(rad), -np.sin(rad)],
        [0, np.sin(rad), np.cos(rad)]
    ])

def rotation_y(angle):
    rad = np.radians(angle)
    return np.array([
        [np.cos(rad), 0, np.sin(rad)],
        [0, 1, 0],
        [-np.sin(rad), 0, np.cos(rad)]
    ])

def rotation_z(angle):
    rad = np.radians(angle)
    return np.array([
        [np.cos(rad), -np.sin(rad), 0],
        [np.sin(rad), np.cos(rad), 0],
        [0, 0, 1]
    ])

def apply_rotations():
    angles = [float(entry_angle1.get()), float(entry_angle2.get()), float(entry_angle3.get())]
    axes = [combo_axis1.get(), combo_axis2.get(), combo_axis3.get()]
    
    p = np.array([2, 0, 1])  # Initial vector
    
    for angle, axis in zip(angles, axes):
        if axis == 'X':
            p = rotation_x(angle) @ p
        elif axis == 'Y':
            p = rotation_y(angle) @ p
        elif axis == 'Z':
            p = rotation_z(angle) @ p
    
    label_result.config(text=f"Final Coordinates: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    plot_3d_transform(angles, axes)

def plot_3d_transform(angles, axes):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Initial basis
    origin = np.array([[0, 0, 0]]).T
    basis = np.eye(3)  # Identity matrix represents the original frame
    
    # Plot initial frame
    colors = ['r', 'g', 'b']
    labels = ['X', 'Y', 'Z']
    for i in range(3):
        ax.quiver(*origin.flatten(), *basis[:, i], color=colors[i], label=f'Initial {labels[i]}')
    
    # Apply rotations step by step
    R = np.eye(3)
    for angle, axis in zip(angles, axes):
        if axis == 'X':
            R = R @ rotation_x(angle)
        elif axis == 'Y':
            R = R @ rotation_y(angle)
        elif axis == 'Z':
            R = R @ rotation_z(angle)
    
    transformed_basis = R @ basis
    for i in range(3):
        ax.quiver(*origin.flatten(), *transformed_basis[:, i], color=colors[i], linestyle='dashed', label=f'Final {labels[i]}')
    
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

# GUI Setup
root = tk.Tk()
root.title("3D Rotation Visualizer")

ttk.Label(root, text="Rotation 1 (deg):").grid(row=0, column=0)
entry_angle1 = ttk.Entry(root)
entry_angle1.grid(row=0, column=1)
combo_axis1 = ttk.Combobox(root, values=['X', 'Y', 'Z'])
combo_axis1.grid(row=0, column=2)


ttk.Label(root, text="Rotation 2 (deg):").grid(row=1, column=0)
entry_angle2 = ttk.Entry(root)
entry_angle2.grid(row=1, column=1)
combo_axis2 = ttk.Combobox(root, values=['X', 'Y', 'Z'])
combo_axis2.grid(row=1, column=2)


ttk.Label(root, text="Rotation 3 (deg):").grid(row=2, column=0)
entry_angle3 = ttk.Entry(root)
entry_angle3.grid(row=2, column=1)
combo_axis3 = ttk.Combobox(root, values=['X', 'Y', 'Z'])
combo_axis3.grid(row=2, column=2)


button_calculate = ttk.Button(root, text="Apply Rotations", command=apply_rotations)
button_calculate.grid(row=3, columnspan=3)

label_result = ttk.Label(root, text="Final Coordinates: ")
label_result.grid(row=4, columnspan=3)

root.mainloop()
