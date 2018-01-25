# import matplotlib
# matplotlib.use('Agg')

# import cvxpy
# import cvxpy as cvx, numpy as np, matplotlib.pyplot as plt
# from qcqp import *

# import cvxopt
# print "cvxopt version = " , cvxpy.__version__

# import mosek
# print "mosek" , mosek
# print('Backend: {}'.format(plt.get_backend()))
# import time
# print "cvxopt version = ", cvxopt.__version__


import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time
show_animation = True
goalSampleRate=20
minrand = -1.5
maxrand = 3

rndIf = random.randint(0, 100)

if rndIf > goalSampleRate:
	rnd = [random.uniform(minrand, maxrand), random.uniform(minrand, 2)]
# else:
	

print "rand in if cond = ", rndIf, "rnd resp = ", rnd
# timeStep = 3
# Ts = .5 # sec !!! 
# d = .8 # m


# rowVehN = 2
# colVehN = timeStep + 1 # initial pose + timeStep positions

# # Initialize matrices
# vehicleN = np.zeros((rowVehN,colVehN),dtype=float)

# # position of vehicle 'n':
# Xninit, Vninit, Yninit = 7,		20,		1.25
# vehicleN[0,0] = Xninit
# vehicleN[1,0] = Yninit

# # position of vehicle n in each time step:
# posNprev = 0
# for i in range(rowVehN):
    # for j in range(1,colVehN):
		# if i == rowVehN - 1:
			# vehicleN[i][j] = Yninit
		# else:
			# vehicleN[i][j] =  (Vninit * Ts) + Xninit # Xn = Vn*Ts + XnPrev!
			# Xninit = vehicleN[i][j] # update the Xn based on previous timeStep
		
# print "vehicle N :", vehicleN
# # replicate = 5

		
# X = cvx.Variable(5*timeStep+3)
# P0 = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)
# q_obj = np.zeros((5*timeStep+3,1),dtype=float)

# q_c = np.zeros((5*timeStep+3,1),dtype=float)
# P_c = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)

# X = cvx.Variable(5*timeStep+3)

# # b = np.zeros((5*timeStep+3,1),dtype=float)

# # A = np.zeros((3*timeStep+3,5*timeStep+3),dtype=float)
# b = np.zeros((3*timeStep+3,1),dtype=float)



# # position of vehicle 'm':
# Xminit, Vminit, Yminit = 0,		22,		17

# # initial control signal, vehicle 'm':
# amInit, VymInit = 0,0
# alpha = .001
# beta = .001
# Amodel = np.matrix([[-1,-Ts,0],
					# [0,-1,0],
					# [0,0,-1]
					# ])
# Bmodel = np.matrix([[-.5*(Ts**2),0],
					# [-Ts,0],
					# [0,-Ts]
					# ])
# Cmodel = np.identity(3)


# # # Vehicle 'm' Initial information:
# # vehMInit = ([
		# # [Xminit],[Vminit],[Yminit],[amInit],[VymInit]
		# # ])

# vehMInit = ([
		# [Xminit],[Vminit],[Yminit]
		# ])


# consTot = [X[0]==vehMInit[0],X[1]==vehMInit[1],X[2]==vehMInit[2]]


# colA = 0
# for idxLinConst in range(timeStep):
	# A = np.zeros((3*timeStep+3,5*timeStep+3),dtype=float)
	# b = np.zeros((3*timeStep+3,1),dtype=float)
	
	# print "index Linear = ", idxLinConst
	
	# rowA = 3*(idxLinConst+1)
	# A[rowA:rowA+3,colA:colA+3] = Amodel
	# colA = colA+3
	# A[rowA:rowA+3,colA:colA+2] = Bmodel
	# colA = colA+2
	# A[rowA:rowA+3,colA:colA+3] = Cmodel
	# print "A for = ", A
	# print "b for = ", b
	# consTot.append(A*X == b)

	
	
