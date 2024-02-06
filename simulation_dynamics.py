'''
***************************************************************************
Modified sovler for simulation
***************************************************************************
Author: Alexandros Mantzios
Credits: Modern Robotics: Mechanics, Planning, and Control. Code Library
Email: alexandrosmantzios@gmail.com
Date: February 2022
***************************************************************************
Language: Python
Required library: numpy, modern_robotics core library
***************************************************************************
'''

'''
*** IMPORTS ***
'''

import numpy as np
import modern_robotics as mr
import math as math
from pathlib import Path
import time

#  Link Frames i relative to i-1 at home position
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]

# Spatial inertia matrices G of links
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])

# List of spatial inertia matrices
Glist = [G1, G2, G3, G4, G5, G6]

# List of link frames
Mlist = [M01, M12, M23, M34, M45, M56, M67] 

# Screw axes Si of joints in space frame for UR5
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

# Select initial thetalist initial joint variables
thetalist = np.array([0,0,0,0,0,0])

# Select initial joint velocities
dthetalist = np.array([0,0,0,0,0,0])

# Select Spatial forces applied by end effector in frame n+1
Ftip = np.array([0,0,0,0,0,0])

# Select n-verctor of joint forces
taulist = np.array([0,0,0,0,0,0])

# Gravity terms
gravity = np.array([0, 0, -9.81])

# Select time interval for 100 iteration per second
dt = 0.001
# Select total iterations
iterations = 3000

result = thetalist.copy()

start_time = time.time()

# Find joint angles and velocities
for i in range(iterations):
	ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, gravity, Ftip, Mlist, Glist, Slist)
	print ("result obtained")
	(thetalist, dthetalist) = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
	result = np.vstack((result, thetalist))
	
	
end_time = time.time()
elapsed_time = end_time - start_time
	
print("Time elapsed:", elapsed_time, "seconds")

# Save iterations on csv file for simulation
np.savetxt("simulation1.csv", result, delimiter=",")


