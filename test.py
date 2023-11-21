import torch
import math
import matplotlib.pyplot as plt


def get_gripper_corners(pose, half_extents_gripper):
    # Calculate the corner points of the rotated box
    half_length, half_height = half_extents_gripper
    center_x, center_y, theta = pose
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    p1 = (center_x - cos_theta * half_length + sin_theta * half_height,
            center_y - sin_theta * half_length - cos_theta * half_height)
    
    p2 = (center_x + cos_theta * half_length + sin_theta * half_height,
            center_y + sin_theta * half_length - cos_theta * half_height)

    p3 = (center_x + cos_theta * half_length - sin_theta * half_height,
            center_y + sin_theta * half_length + cos_theta * half_height)

    p4 = (center_x - cos_theta * half_length - sin_theta * half_height,
            center_y - sin_theta * half_length + cos_theta * half_height)

    # p1 = (center_x - cos_theta * half_length + sin_theta * half_height,
    #         center_y + sin_theta * half_length + cos_theta * half_height)
    
    # p2 = (center_x + cos_theta * half_length + sin_theta * half_height,
    #         center_y - sin_theta * half_length + cos_theta * half_height)

    # p3 = (center_x + cos_theta * half_length - sin_theta * half_height,
    #         center_y - sin_theta * half_length - cos_theta * half_height)

    # p4 = (center_x - cos_theta * half_length - sin_theta * half_height,
    #         center_y + sin_theta * half_length - cos_theta * half_height)
    
    return torch.tensor([p1, p2, p3, p4]) # 4x2

def draw_gripper(corners, ax, alpha=0.5):
    points = corners # 4x2
    x, y = points[:, 0], points[:, 1]
    for i in range(len(points)):
        next_i = (i + 1) % len(points)  # To loop back to the first point
        ax.plot([x[i], x[next_i]], [y[i], y[next_i]], 'g-', alpha=alpha)  # 'b-' for blue lines

pose = [2.5883,  6.4483, -0.2511]
half_extents_gripper = [.5,.1]
corners = get_gripper_corners(pose, half_extents_gripper)

fig, ax = plt.subplots()

draw_gripper(corners, ax, alpha=float(.5))

ax.set_xlim([0.,10.0])
ax.set_ylim([0.,10.0])
ax.legend(loc="upper right")
ax.set_aspect("equal")
plt.tight_layout()
plt.show()