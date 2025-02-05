import numpy as np

def find_contact_point_with_centroid(triangle, force_direction, centroid):
    """
    Find the first contact point of an external force line with a triangle.
    Assumes the centroid is known but not zero.

    Parameters:
        triangle: List of tuples [(x1, y1), (x2, y2), (x3, y3)] representing the triangle vertices.
        force_direction: Tuple (fx, fy), the direction vector of the external force.
        centroid: Tuple (Gx, Gy), the known centroid of the triangle.

    Returns:
        Tuple (x, y): The first contact point, or None if no valid intersection occurs.
    """
    Gx, Gy = centroid  # Known centroid
    F = np.array(force_direction)  # Force vector (direction)
    closest_t = float('inf')  # Track the smallest positive t (closest intersection)
    contact_point = None

    for i in range(len(triangle)):
        # Get edge vertices
        x_i, y_i = triangle[i]
        x_j, y_j = triangle[(i + 1) % len(triangle)]

        # Calculate s
        denominator = F[1] * (x_j - x_i) - F[0] * (y_j - y_i)
        if np.isclose(denominator, 0):
            continue  # Parallel lines

        numerator = F[0] * (y_i - Gy) - F[1] * (x_i - Gx)
        s = numerator / denominator

        if 0 <= s <= 1:
            # Intersection point on the edge
            x_int = x_i + s * (x_j - x_i)
            y_int = y_i + s * (y_j - y_i)

            # Calculate t for force direction
            t = ((x_int - Gx) * F[0] + (y_int - Gy) * F[1]) / (F[0]**2 + F[1]**2)

            if t > 0 and t < closest_t:
                closest_t = t
                contact_point = (x_int, y_int)

    return contact_point

# Example usage
triangle = [(1, 1), (3, 1), (2, 3)]  # Triangle vertices
force_direction = (-1, -1)  # External force direction vector
centroid = (2, 1.6667)  # Known centroid of the triangle

contact_point = find_contact_point_with_centroid(triangle, force_direction, centroid)
print("First contact point:", contact_point)