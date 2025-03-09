import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

def rotation_x(theta):
    theta = np.radians(theta)
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def rotation_y(theta):
    theta = np.radians(theta)
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rotation_z(theta):
    theta = np.radians(theta)
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def plot_axes(ax):
    ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X-axis')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y-axis')
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z-axis')
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

def update_plot():
    try:
        x_angle = float(entry_x.get())
        y_angle = float(entry_y.get())
        z_angle = float(entry_z.get())
    except ValueError:
        return
    
    p = np.array([2, 0, 1])
    p_rotated = rotation_z(z_angle) @ rotation_y(y_angle) @ rotation_x(x_angle) @ p
    
    ax.clear()
    plot_axes(ax)
    ax.scatter(*p, color='black', s=50, label='Original Point')
    ax.scatter(*p_rotated, color='magenta', s=50, label='Rotated Point')
    ax.legend()
    canvas.draw()
    
    result_label.config(text=f"Rotated Point: ({p_rotated[0]:.2f}, {p_rotated[1]:.2f}, {p_rotated[2]:.2f})")

# GUI setup
root = tk.Tk()
root.title("3D Rotation Visualization")

frame = ttk.Frame(root, padding=10)
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# Input fields
entry_x = ttk.Entry(frame, width=5)
entry_y = ttk.Entry(frame, width=5)
entry_z = ttk.Entry(frame, width=5)

entry_x.grid(row=0, column=1)
entry_y.grid(row=1, column=1)
entry_z.grid(row=2, column=1)

# Labels
ttk.Label(frame, text="X Rotation (°):").grid(row=0, column=0)
ttk.Label(frame, text="Y Rotation (°):").grid(row=1, column=0)
ttk.Label(frame, text="Z Rotation (°):").grid(row=2, column=0)

# Button
btn_rotate = ttk.Button(frame, text="Rotate", command=update_plot)
btn_rotate.grid(row=3, columnspan=2, pady=5)

# Result label
result_label = ttk.Label(frame, text="Rotated Point: ( , , )", font=("Arial", 12))
result_label.grid(row=4, columnspan=2)

# 3D plot setup
fig = plt.figure(figsize=(5, 5))
ax = fig.add_subplot(111, projection='3d')
plot_axes(ax)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().grid(row=0, column=1)

root.mainloop()
