import numpy as np

# ------------------------------
# Functions to construct rotation matrices
# ------------------------------
def rot_x(angle):
    """Returns a rotation matrix for a rotation about the X-axis by 'angle' radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c]
    ])

def rot_y(angle):
    """Returns a rotation matrix for a rotation about the Y-axis by 'angle' radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(angle):
    """Returns a rotation matrix for a rotation about the Z-axis by 'angle' radians."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [ c, -s, 0],
        [ s,  c, 0],
        [ 0,  0, 1]
    ])

# ------------------------------
# Function to construct a rotation matrix from Euler angles
# ------------------------------
def euler_to_rot_matrix(angles, order="zyx"):
    """
    Constructs a rotation matrix from a given set of Euler angles.

    Parameters:
        angles: tuple/list of angles (in radians)
        order: string specifying the Euler sequence ("zyx", "xzx", etc.)

    Returns:
        A 3x3 NumPy rotation matrix.
    """
    if len(angles) != len(order):
        raise ValueError("The number of angles must match the length of the order string.")
    
    rot_funcs = {'x': rot_x, 'y': rot_y, 'z': rot_z}
    R = np.eye(3)

    for axis, angle in zip(order.lower(), angles):
        if axis not in rot_funcs:
            raise ValueError("Invalid axis in order string. Use only 'x', 'y', or 'z'.")
        R = R @ rot_funcs[axis](angle)

    return R

# ------------------------------
# Functions to extract Euler angles from a rotation matrix
# ------------------------------

def rot_matrix_to_euler_xyz(R):
    """Extracts Euler angles from a rotation matrix using the XYZ convention."""
    if abs(R[0, 2]) < 1:
        theta = np.arcsin(-R[0, 2])
        cos_theta = np.cos(theta)
        phi = np.arctan2(R[1, 2] / cos_theta, R[2, 2] / cos_theta)
        psi = np.arctan2(R[0, 1] / cos_theta, R[0, 0] / cos_theta)
    else:  
        # Gimbal lock case
        phi = 0  
        if R[0, 2] == -1:
            theta = np.pi / 2
            psi = np.arctan2(R[1, 0], R[1, 1])
        else:
            theta = -np.pi / 2
            psi = np.arctan2(-R[1, 0], -R[1, 1])

    return (phi, theta, psi)

def rot_matrix_to_euler_zyx(R):
    """Extracts Euler angles from a rotation matrix using the ZYX convention."""
    if abs(R[2, 0]) < 1:
        theta = -np.arcsin(R[2, 0])
        cos_theta = np.cos(theta)
        psi = np.arctan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
        phi = np.arctan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
    else:
        # Gimbal lock condition
        phi = 0  
        if R[2, 0] == -1:
            theta = np.pi / 2
            psi = np.arctan2(R[0, 1], R[0, 2])
        else:
            theta = -np.pi / 2
            psi = np.arctan2(-R[0, 1], -R[0, 2])

    return (phi, theta, psi)

def rot_matrix_to_euler_zxz(R):
    """Extracts Euler angles from a rotation matrix using the ZXZ convention."""
    if abs(R[2, 2]) < 1:
        theta = np.arccos(R[2, 2])
        sin_theta = np.sin(theta)
        phi = np.arctan2(R[0, 2] / sin_theta, -R[1, 2] / sin_theta)
        psi = np.arctan2(R[2, 0] / sin_theta, R[2, 1] / sin_theta)
    else:  
        # Gimbal lock case
        theta = 0 if R[2, 2] == 1 else np.pi
        phi = np.arctan2(R[0, 1], R[0, 0])
        psi = 0  

    return (phi, theta, psi)

def rot_matrix_to_euler_zyz(R):
    """Extracts Euler angles from a rotation matrix using the ZYZ convention."""
    if abs(R[2, 2]) < 1:
        theta = np.arccos(R[2, 2])
        sin_theta = np.sin(theta)
        phi = np.arctan2(R[1, 2] / sin_theta, R[0, 2] / sin_theta)
        psi = np.arctan2(R[2, 1] / sin_theta, -R[2, 0] / sin_theta)
    else:  
        # Gimbal lock case
        theta = 0 if R[2, 2] == 1 else np.pi
        phi = np.arctan2(R[0, 1], R[0, 0])
        psi = 0  

    return (phi, theta, psi)

def rot_matrix_to_euler_xzx(R):
    """Extracts Euler angles from a rotation matrix using the XZX convention."""
    if abs(R[0, 0]) < 1:
        theta = np.arccos(R[0, 0])
        sin_theta = np.sin(theta)
        phi = np.arctan2(R[2, 0] / sin_theta, R[1, 0] / sin_theta)
        psi = np.arctan2(R[0, 2] / sin_theta, -R[0, 1] / sin_theta)
    else:
        # Gimbal lock case
        theta = 0 if R[0, 0] == 1 else np.pi
        phi = np.arctan2(R[1, 2], R[1, 1])
        psi = 0  

    return (phi, theta, psi)

# ------------------------------
# Unified function for extracting Euler angles from any order
# ------------------------------
def rot_matrix_to_euler(R, order):
    """
    Extracts Euler angles from a given rotation matrix based on the specified order.

    Supported orders:
        - "xyz"
        - "zyx"
        - "zxz"
        - "zyz"
        - "xzx"

    Returns:
        A tuple of three Euler angles in radians.
    """
    order = order.lower()
    if order == "xyz":
        return rot_matrix_to_euler_xyz(R)
    elif order == "zyx":
        return rot_matrix_to_euler_zyx(R)
    elif order == "zxz":
        return rot_matrix_to_euler_zxz(R)
    elif order == "zyz":
        return rot_matrix_to_euler_zyz(R)
    elif order == "xzx":
        return rot_matrix_to_euler_xzx(R)
    else:
        raise ValueError(f"Unsupported rotation order: {order}")

# ------------------------------
# Main function to test Zyx and Xzx transformations
# ------------------------------
def main():
    """Computes a rotation matrix using Zyx convention and extracts equivalent Xzx angles."""
    
    # Given Euler angles in Zyx convention (in degrees)
    psi_deg, theta_deg, phi_deg = 30, 60, 45  # psi (Z), theta (Y), phi (X)

    print("\nGiven Euler angles (degrees) [psi, theta, phi]:")
    print([psi_deg, theta_deg, phi_deg])

    # Convert to radians
    angles_rad = np.radians([psi_deg, theta_deg, phi_deg])
    
    # Construct the rotation matrix for Zyx order
    R_zyx = euler_to_rot_matrix(angles_rad, "zyx")
    
    print("\nRotation Matrix (constructed using ZYX convention):\n", R_zyx)
    
    # Extract Euler angles using Zyx convention
    euler_zyx = rot_matrix_to_euler_zyx(R_zyx)
    euler_zyx_deg = np.degrees(euler_zyx)
    
    print("\nExtracted Euler angles (ZYX convention) [phi, theta, psi] in degrees:")
    print(euler_zyx_deg)

    # Verify reconstruction
    R_zyx_reconstructed = euler_to_rot_matrix(np.radians(euler_zyx_deg), "zyx")

    print("\nReconstructed Rotation Matrix from extracted ZYX angles:")
    print(R_zyx_reconstructed)

    # Compute reconstruction error
    error_zyx = np.linalg.norm(R_zyx - R_zyx_reconstructed)
    print("\nReconstruction Error (ZYX):", error_zyx)

    # ----------------- XZX EXTRACTION ----------------- #

    # Extract Euler angles using Xzx convention
    euler_xzx = rot_matrix_to_euler_xzx(R_zyx)
    euler_xzx_deg = np.degrees(euler_xzx)
    
    print("\nExtracted Euler angles (XZX convention) [phi, theta, psi] in degrees:")
    print(euler_xzx_deg)

    # Verify reconstruction
    R_xzx_reconstructed = euler_to_rot_matrix(np.radians(euler_xzx_deg), "xzx")

    print("\nReconstructed Rotation Matrix from extracted XZX angles:")
    print(R_xzx_reconstructed)

    # Compute reconstruction error
    error_xzx = np.linalg.norm(R_zyx - R_xzx_reconstructed)
    print("\nReconstruction Error (XZX):", error_xzx)

# ------------------------------
# Run the main function
# ------------------------------
if __name__ == "__main__":
    main()
