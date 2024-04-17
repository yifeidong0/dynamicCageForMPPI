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

def sample_points_in_ellipsoid_3d(center, radii, rotation, num_samples=100):
    # Generate random points in a unit sphere using spherical coordinates
    u = np.random.uniform(0, 1, num_samples)
    theta = np.random.uniform(0, 2 * np.pi, num_samples)
    phi = np.arccos(1 - 2 * u)
    r = np.cbrt(np.random.uniform(0, 1, num_samples))  # Cube root for volume uniformity

    # Convert spherical to cartesian coordinates
    points = np.vstack((r * np.sin(phi) * np.cos(theta), r * np.sin(phi) * np.sin(theta), r * np.cos(phi))).T

    # Scale points by the radii of the ellipsoid
    points *= radii

    # Rotate points to align with the ellipsoid's axes
    points = np.dot(points, rotation.T)

    # Translate points to the center of the ellipsoid
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

from mpl_toolkits.mplot3d import Axes3D
def plot_ellipsoid_and_points(center, radii, rotation, points, P):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the original points
    ax.scatter(P[:, 0], P[:, 1], P[:, 2], color='blue', s=50, label='Original Points')

    # Generate a mesh for the ellipsoid
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))

    # Transform each point on the mesh
    for i in range(len(x)):
        for j in range(len(x[0])):
            [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation.T) + center

    # Plot the ellipsoid
    ax.plot_surface(x, y, z, rstride=4, cstride=4, color='b', alpha=0.25, label='Bounding Ellipsoid')

    # Plot sampled points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', s=20, label='Sampled Points')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    plt.title('Minimum Bounding Ellipsoid with Sampled Points')
    # plt.legend()
    plt.show()


# # Example usage:
# P = np.array([[1, 1], [2, -1], [1, 3], [3, 2], [0.5, 1.5]])
# center, radii, rotation = khachiyan_algorithm(P)
# points = sample_points_in_ellipse(center, radii, rotation, num_samples=5)
# plot_ellipse_and_points(center, radii, rotation, points, P)
    
# # Example usage for 3D ellipsoid:
# P = np.array([[1, 1, 1], [2, -1, 0], [1, 3, 2], [3, 2, -1], [0.5, 1.5, 1]])
# center, radii, rotation = khachiyan_algorithm(P)
# points = sample_points_in_ellipsoid_3d(center, radii, rotation, num_samples=300)
# plot_ellipsoid_and_points(center, radii, rotation, points, P)