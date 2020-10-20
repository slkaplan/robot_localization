#!/usr/bin/env python3

import numpy as np

num_particles = 2
angles = np.array([0, 90, 180, 270]) # will be scan indices (0-361)
distances = np.array([0,1,2,3]) # will be scan values (scan)
angles_rad = np.deg2rad(angles)

print(angles)
sin_values = np.sin(angles)
cos_values = np.cos(angles)
d_angles = np.multiply(distances, sin_values)
print(d_angles)
p = np.full((10,4),d_angles)
transpose = np.transpose(p)
print()
print(p)
print()
print(transpose)