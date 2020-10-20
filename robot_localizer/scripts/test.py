#!/usr/bin/env python3

import numpy as np

# distances = np.array(scan)
# angles = np.deg2rad(np.arange(361))
# cos_values = np.cos(angles)
# sin_values = np.sin(angles)

num_particles = 2
angles = np.array([0, 90, 180, 270])
distances = np.array([0,1,2,3])
angles_rad = np.deg2rad(angles)
print(angles)
sin_values = np.sin(angles)
cos_values = np.cos(angles)
d_angles = np.multiply(distances, sin_values)
print(d_angles)