# Author - navneet0401

from __future__ import division

from sympy import *

f_n, f_n1, f_n2, f_n3, f_n4 = symbols('f_n f_n1 f_n2 f_n3 f_n4')
# f_n is the nth value -> f(tn)
# f_n1 is the value at t_(n-1) and so on

A, B, C, D = symbols('A B C D')
dt = symbols('dt') # dt = t_(n-1) - tn

# Taylor Series Expansion to four degree diff
eq1 = f_n - (A*dt) + (B*(dt)**2)/2 - (C*(dt)**3)/6 + (D*(dt)**4)/24 - f_n1
eq2 = f_n - (A*2*dt) + (B*(2*dt)**2)/2 - (C*(2*dt)**3)/6 + (D*(2*dt)**4)/24 - f_n2
eq3 = f_n - (A*3*dt) + (B*(3*dt)**2)/2 - (C*(3*dt)**3)/6 + (D*(3*dt)**4)/24 - f_n3
eq4 = f_n - (A*4*dt) + (B*(4*dt)**2)/2 - (C*(4*dt)**3)/6 + (D*(4*dt)**4)/24 - f_n4
sol = linsolve([eq1, eq2, eq3, eq4], A,B,C,D)

vel, acc, jerk, d = next(iter(sol))
print(vel)
print(acc)