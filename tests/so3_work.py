import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_work(alpha, I, x, xnext):
    """
    Calculate the work done by applying torque to a rigid object.

    Parameters:
    alpha (np.array): The angular acceleration applied (3 elements).
    I (np.array): The moment of inertia of the object (3x3 matrix).
    x (np.array): The initial Euler angles of the object (3 elements).
    xnext (np.array): The final Euler angles of the object (3 elements).

    Returns:
    float: The work done on the object.
    """
    # Convert Euler angles to quaternions
    q1 = R.from_euler('xyz', x).as_quat()
    q2 = R.from_euler('xyz', xnext).as_quat()

    # Calculate the relative rotation quaternion
    dq = R.from_quat(q2) * R.from_quat(q1).inv()

    # Convert the relative rotation to an angular displacement vector
    theta = dq.as_rotvec()

    # Calculate the work done as the dot product of torque and angular displacement
    work = np.dot(torque, theta)

    return work

# Example usage:
torque = np.array([1, 0, 0])  # Apply torque around the x-axis
I = np.diag([1, 2, 3])  # Moment of inertia for the object
x = np.array([np.pi/2+.3, np.pi/4+.2, np.pi+.5])  # Initial Euler angles (object orientation)
xnext = np.array([np.pi/2, np.pi/4, np.pi])  # Final Euler angles after applying the torque

work_done = calculate_work(torque, I, x, xnext)
print("Work done on the object:", work_done)