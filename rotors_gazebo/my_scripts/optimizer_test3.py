import math
import numpy as np
from sympy import symbols

dji = np.mat(np.array([90, 30, 0]))
vj = np.mat(np.array([-10, 0, 0]))

vix, viy, viz = symbols('vix, viy, viz')
vi = np.mat(np.array([10, 10, 0]))

v = vi - vj

v_inv = np.mat(np.array([np.linalg.pinv(v)]))

print(dji.shape)
print(v_inv.T.shape)

tf = dji * v_inv.T
print(float(tf))
