import matplotlib
matplotlib.use('Agg')

import cvxpy as cvx
import numpy as np
import matplotlib.pyplot as plt
from qcqp import *
import mosek

n = 5 # number of circles
X = cvx.Variable(2, n)
B = 10

r = cvx.Variable()
obj = cvx.Maximize(r)
cons = [X >= r, X <= B-r, r >= 0]
for i in range(n):
    for j in range(i+1, n):
        cons.append(cvx.square(2*r) <= cvx.sum_squares(X[:, i]-X[:, j]))

prob = cvx.Problem(obj, cons)

# Create a QCQP handler.
qcqp = QCQP(prob)



# sample from the semidefinite relaxation
# qcqp.suggest(SDR, solver=cvx.MOSEK)

# f_cd, v_cd = qcqp.improve(COORD_DESCENT)
# print("Coordinate descent: objective %.3f, violation %.3f" % (f_cd, v_cd))

# SDR solution is cached and not solved again
qcqp.suggest(SDR, solver=cvx.MOSEK)

sdrBound = qcqp.sdr_bound
sdrSol = qcqp.sdr_sol

print("SDR: upper bound: %.3f" % sdrBound)
print "solution = ", sdrSol
X_dccpSug = np.copy(X.value)
r_dccpSug = r.value


print "X dccp Suggest = ", X_dccpSug
print "r_dccp Suggest = ", r_dccpSug

f_dccp, v_dccp = qcqp.improve(DCCP)
print("Penalty CCP: objective %.3f, violation %.3f" % (f_dccp, v_dccp))

X_dccp = np.copy(X.value)
r_dccp = r.value


print "X dccp = ", X_dccp
print "r_dccp = ", r_dccp


# qcqp.suggest(SDR)
# f_admm, v_admm = qcqp.improve(ADMM)

# print("Nonconvex ADMM: objective %.3f, violation %.3f" % (f_admm, v_admm))
# X_admm = np.copy(X.value)
# r_admm = r.value

# print "X admm = ", X_admm
# print "r_admm = ", r_admm


# plot the circles
circ = np.linspace(0, 2*np.pi)

plt.figure()
for i in xrange(n):
    plt.plot(X_dccp[0, i]+r_dccp*np.cos(circ), X_dccp[1, i]+r_dccp*np.sin(circ), 'b')

# plt.xlim([0, B])
# plt.ylim([0, B])
plt.axes().set_aspect('equal')
plt.savefig('result.jpeg', dpi = 500)

plt.figure()
for i in xrange(n):
    plt.plot(X_dccpSug[0, i]+r_dccpSug*np.cos(circ), X_dccpSug[1, i]+r_dccpSug*np.sin(circ), 'b')

# plt.xlim([0, B])
# plt.ylim([0, B])
plt.axes().set_aspect('equal')
plt.savefig('resultSuggest.jpeg', dpi = 500)

plt.show()