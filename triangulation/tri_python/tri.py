import numpy as np

def triangulate(points2d, poses):
    """
    Triangulate a 3D point from a list of 2D points and camera poses.

    Args:
        points2d: a list of 2D points, expressed as (x, y) coordinates
        poses: a list of camera poses, expressed as (R, t) matrices

    Returns:
        a 3D point, expressed as a NumPy array
    """

    # Create a matrix A where each row is the coordinates of a point
    # expressed in the coordinate system of the corresponding pose
    A = []
    for i in range(len(points2d)):
        p = points2d[i]
        R, t = poses[i]
        A.append(np.dot(R, p) + t)

    # Solve the linear system of equations A * x = 0 to find the
    # 3D point that minimizes the distance to all the 2D points
    x = np.linalg.solve(A, np.zeros(3))

    return x


