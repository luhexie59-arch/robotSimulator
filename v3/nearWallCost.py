'''
These functions are experimental only at present.
'''

import numpy as np
def nearWallCostFunctions(x, y, method:str, map, sensitive_radius):
    if method == "binary":
        return binaryCost(x, y, map, sensitive_radius)
    elif method == "gaussian":
        return GaussianDistanceCost(x, y, map, sensitive_radius)
    elif method == "inverse":
        return InverseDistanceCost(x, y, map, sensitive_radius)
    else:
        raise ValueError("Unknown method")
    

def binaryCost(x, y, map, sensitive_radius):
    if np.any(map[x-sensitive_radius:x+sensitive_radius+1, y-sensitive_radius:y+sensitive_radius+1] == 1):
        return 100
    return 0

def GaussianDistanceCost(x, y, map, sensitive_radius):
    x0, x1 = x-sensitive_radius, x+sensitive_radius+1
    y0, y1 = y-sensitive_radius, y+sensitive_radius+1
    surround = map[x0:x1, y0:y1]
    distance = np.exp(-0.3 * np.indices(surround.shape))  # Gaussian falloff
    return np.sum(surround * distance)

def InverseDistanceCost(x, y, map, sensitive_radius):
    x0, x1 = x-sensitive_radius, x+sensitive_radius+1
    y0, y1 = y-sensitive_radius, y+sensitive_radius+1
    surround = map[x0:x1, y0:y1]
    dist_matrix = np.indices(surround.shape)
    dist_matrix = np.sqrt(np.sum(dist_matrix**2, axis=0))

    return np.sum(surround / (dist_matrix + 1))
