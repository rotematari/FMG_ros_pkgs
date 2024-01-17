import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_euler_angles(vector):
    x1, y1, z1 = vector
    x1, y1, z1 = x1 / np.linalg.norm(vector), y1 / np.linalg.norm(vector), z1 / np.linalg.norm(vector)
    pitch = np.arctan2(-x1, np.sqrt(y1**2 + z1**2))
    yaw = np.arctan2(y1, x1)
    roll = 0  # This requires additional reference, like an 'up' vector
    return roll, pitch, yaw


def calculate_full_circle_angles(vector):
    # Unit vectors for the x, y, and z axes
    unit_vectors = {
        'X': np.array([1, 0, 0]),
        'Y': np.array([0, 1, 0]),
        'Z': np.array([0, 0, 1])
    }
    
    angles = {}
    vector_magnitude = np.linalg.norm(vector)

    for axis, unit_vector in unit_vectors.items():
        # Calculate the dot product
        dot_product = np.dot(vector, unit_vector)
        # Calculate the angle in radians between 0 and π
        angle = np.arccos(np.clip(dot_product / vector_magnitude, -1.0, 1.0))
        # Calculate the cross product to determine the sign
        cross_product = np.cross(vector, unit_vector)
        # Determine if the angle is beyond π (180 degrees) using the cross product
        if cross_product[{'X': 1, 'Y': 2, 'Z': 0}[axis]] < 0:  # Change the index based on the axis
            angle = 2 * np.pi - angle
        angles[axis] = angle
    
    return angles

def calculate_angles_with_axes(vector):

    unit_vectors = {
    'X': np.array([1, 0, 0]),
    'Y': np.array([0, 1, 0]),
    'Z': np.array([0, 0, 1])
    }
    
    angles = {axis: [] for axis in unit_vectors}  # Initialize as a dictionary of lists

    vector = np.array(vector)
    vector_magnitude = np.linalg.norm(vector)

    # Check for zero magnitude to avoid division by zero
    if vector_magnitude == 0:
        for axis in unit_vectors:
            angles[axis].append(0)
        

    for axis, unit_vector in unit_vectors.items():
        dot_product = np.dot(vector, unit_vector)
        # Clamp the value to the valid range for arccos to avoid numerical errors
        cos_angle = dot_product / vector_magnitude
        print(cos_angle)
        angle_rad = np.arccos(abs(cos_angle))
        angles[axis].append(angle_rad)

    return angles

def calculate_vectors(locatios: dict):
    MC = np.array(locatios['chest'][0])
    MS = np.array(locatios['shoulder'][0])
    ME = np.array(locatios['elbow'][0])
    MW = np.array(locatios['wrist'][0])
    MT = np.array(locatios['table_base'][0])
    # print(MS)
    # print(ME)
    CtoS = MS - MC
    StoE = ME - MS
    EtoW = MW - ME
    CtoS, StoE, EtoW = CtoS/np.linalg.norm(CtoS), StoE/np.linalg.norm(StoE), EtoW/np.linalg.norm(EtoW)
    # print(StoE)
    return CtoS, StoE, EtoW

def normalize(v):
    """Normalize a vector."""
    return v / np.linalg.norm(v)

def rotation_matrix_from_vectors(vec1, vec2):
    """Find the rotation matrix that aligns vec1 to vec2."""
    a, b = normalize(vec1), normalize(vec2)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def euler_from_matrix(vec1, vec2):
    """Convert rotation matrix to Euler angles."""
    matrix = rotation_matrix_from_vectors(vec1,vec2)
    sy = np.sqrt(matrix[0, 0] ** 2 +  matrix[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(matrix[2, 1], matrix[2, 2])
        y = np.arctan2(-matrix[2, 0], sy)
        z = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        x = np.arctan2(-matrix[1, 2], matrix[1, 1])
        y = np.arctan2(-matrix[2, 0], sy)
        z = 0
    return np.array([x, y, z])



# if __name__=="__main__":

