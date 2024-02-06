'''
***************************************************************************
5th order polynomial path solver
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


import sympy as sp


# Define the symbolic variable T
T = sp.symbols('T')

# Define the coefficients as symbolic variables
a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')

# Define the 5th order polynomial equation
s_T = a5 * T**5 + a4 * T**4 + a3 * T**3 + a2 * T**2 + a1 * T + a0

# Define initial conditions 
initial_conditions = [
    s_T.subs(T, 0)- 0,       # s(0) = 0
    diff(s_T, T).subs(T, 0)- 0,  # s'(0) = 0
    diff(s_T, T, 2).subs(T, 0)- 0,  # s''(0) = 0
    s_T.subs(T, 1)- 1,       # s(T) = 1
    diff(s_T, T).subs(T, 1)- 0,  # s'(T) = 0
    diff(s_T, T, 2).subs(T, 1)- 0  # s''(T) = 0
]

# Solve the system of equations
coefficients = sp.solve(initial_conditions, (a0, a1, a2, a3, a4, a5))

# Print the coefficients
print(coefficients)