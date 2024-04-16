import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from numpy.linalg import eig, inv

def khachiyan_algorithm(P, tolerance=1e-6):
    (n, d) = np.shape(P)
    Q = np.vstack([np.copy(P.T), np.ones(n)])
    QT = Q.T
    err = 1.0 + tolerance
    u = np.ones(n) / n
    while err > tolerance:
        X = np.dot(np.dot(Q, np.diag(u)), QT)
        M = np.diag(np.dot(np.dot(QT, inv(X)), Q))
        j = np.argmax(M)
        maximum = M[j]
        step_size = (maximum - d - 1) / ((d + 1) * (maximum - 1))
        new_u = (1 - step_size) * u
        new_u[j] += step_size
        err = np.linalg.norm(new_u - u)
        u = new_u
    center = np.dot(u, P)
    A = inv(np.dot(np.dot(P.T, np.diag(u)), P) - np.outer(center, center)) / d
    eigenvalues, eigenvectors = eig(A)
    radii = 1. / np.sqrt(eigenvalues)
    return center, radii, eigenvectors

def sample_points_in_ellipse(center, radii, rotation, num_samples=100):
    # Generate random points in a unit circle
    angles = np.random.uniform(0, 2 * np.pi, num_samples)
    r = np.sqrt(np.random.uniform(0, 1, num_samples))  # Square root of uniform distribution for area uniformity
    points = np.vstack((r * np.cos(angles), r * np.sin(angles))).T

    # Scale points to the ellipse using radii
    points *= radii

    # Rotate points according to the ellipse's rotation matrix
    points = np.dot(points, rotation.T)

    # Translate points by the center
    points += center

    return points

def plot_ellipse_and_points(center, radii, rotation, points, P):
    fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})
    ellipse = Ellipse(xy=center, width=2*radii[0], height=2*radii[1], angle=np.degrees(np.arctan2(*rotation[:,0][::-1])))
    ax.add_patch(ellipse)
    ellipse.set_clip_box(ax.bbox)
    ellipse.set_alpha(0.5)
    ellipse.set_facecolor(np.random.rand(3))
    ax.set_xlim(center[0] - radii[0] - 1, center[0] + radii[0] + 1)
    ax.set_ylim(center[1] - radii[1] - 1, center[1] + radii[1] + 1)
    plt.scatter(points[:, 0], points[:, 1], color='red', s=10)
    plt.scatter(P[:, 0], P[:, 1], color='blue', s=30)
    plt.grid(True)
    plt.show()

# # Example usage:
# P = np.array([[1, 1], [2, -1], [1, 3], [3, 2], [0.5, 1.5]])
# center, radii, rotation = khachiyan_algorithm(P)
# points = sample_points_in_ellipse(center, radii, rotation, num_samples=5)
# plot_ellipse_and_points(center, radii, rotation, points, P)