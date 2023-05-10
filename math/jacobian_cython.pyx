# jacobian_cython.pyx
cimport cython
from sympy import lambdify

cpdef compute_jacobian(functions, variables):
    jacobian = functions.jacobian(variables)
    return lambdify(variables, jacobian)
