import cvxpy as cp
import math
import numpy as np

yj = 0
yi = 0
xj = 50
xi = -50
vj = 10
vi = 10

phi_in_deg = 90
phi_in_rad = phi_in_deg * math.pi / 180
sinphi = math.sin(phi_in_rad)
cosphi = math.cos(phi_in_rad)

sintheta = cp.Variable()
costheta = cp.Variable()

objective = cp.Minimize(((yj - yi)/((vi * sintheta) - (vj * sinphi))) + ((xj - xi)/((vi * costheta) - (vj * cosphi))))

# objective_y = cp.Minimize((yj - yi)/((vi * sintheta) - (vj * sinphi)))
# objective_x = cp.Minimize((xj - xi)/(-(vi * costheta) - (vj * cosphi)))

# constraints_x = [costheta <= 1, costheta >= -0.999999]
# constraints_y = [sintheta <= 1, sintheta >= -0.999999]
constraints = [costheta <= 1, costheta >= -0.999999, sintheta <= 1, sintheta >= -0.999999]

# problem_y = cp.Problem(objective_y, constraints_y)
# problem_x = cp.Problem(objective_x, constraints_x)
problem = cp.Problem(objective, constraints)

# print("Min time y = ", problem_y.solve(qcp=True))
# print("Min time x = ", problem_x.solve(qcp=True))

print(problem.solve())

# print("sintheta Angle = ", 180/math.pi * math.asin(sintheta.value))
# print("costheta Angle = ", 180/math.pi * math.acos(costheta.value))

