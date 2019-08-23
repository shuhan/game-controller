import numpy as np


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
            >>> + identifies counter colockwise rotation - identifies clock wise rotation
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    sign = 1
    if np.dot(v1_u, v2_u) < 0:
        sign = -1
    return sign * np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


front   = np.array([2, 2])
centre  = np.array([3, 1])

drone   = np.array([2, 4])


v1      = front - centre
v2      = drone - centre

print(np.degrees(angle_between(v1, v2)))
print(np.linalg.norm(v2))