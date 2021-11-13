import numpy as np

# Number of terms in Taylor Expansion to approximate f(x)
n = 5
# f(x) \approx f(x_0) + f^1(x_0) * (x-x_0) + f^2(x_0) * (x-x_0)^2 / 2! + ... + f^(n-1)(x_0) * (x-x_0)^(n-1) / (n-1)!
# f(x_0 - D * i) \approx f(x_0) - f^1(x_0) * D * i + f^2(x_0) * D^2 * i^2 / 2! + ... + f^(n-1) * D^(n-1) * i^(n-1) / (n-1)!

# Coefficient matrix A to represent 
# [f(x_0) f(x_1) f(x_2) ... f(x_(n-1))]^T = A . [f(x_0)  f^1(x_0) * D  f^2(x_0) * D^2  ...  f^(n-1)(x_0) * D^(n-1)]^T
# x_i = x_0 - i * D
A = np.zeros((n,n), dtype=float)

A[:, 0] = 1.0

i = np.linspace(0, n, n, False)

factorial = 1

for j in range(1, n) :

	factorial *= j

	A[:, j] = (-i) ** j / factorial

A_inv = np.linalg.inv(A)

print('Coefficients for First Derivative')

print(A_inv[1,:])

print('Coefficients for Second Derivative')

print(A_inv[2,:])

