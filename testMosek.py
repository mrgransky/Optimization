# import matplotlib
# matplotlib.use('Agg')

import cvxpy
import cvxpy as cvx, numpy as np, matplotlib.pyplot as plt
from qcqp import *

import cvxopt
print ("cvx version = ")
print cvxpy.__version__

import pickle
import mosek

import time

print "cvxopt version = ", cvxopt.__version__
from numpy import arange


plt.figure()
plt.plot([1,2], [1,2])

# Option 1
# QT backend
manager = plt.get_current_fig_manager()
manager.window.showMaximized()

# Option 2
# TkAgg backend
manager = plt.get_current_fig_manager()
manager.resize(*manager.window.maxsize())

# Option 3
# WX backend
manager = plt.get_current_fig_manager()
manager.frame.Maximize(True)

plt.show()
plt.savefig('sampleFileName.png')


timeStep = 10
Ts = .5 # sec !!! 
d = .8 # m


rowVehN = 2
colVehN = timeStep + 1 # initial pose + timeStep positions

# Initialize matrices
vehicleN = np.zeros((rowVehN,colVehN),dtype=float)

# fitness_list = arange(1, 50, 2)
# trait1_list = arange(0, 250, 10)
# trait2_list = arange(150, 0, -6)

# position of vehicle 'n':
Xninit, Vninit, Yninit = 7,		20,		1.25
vehicleN[0,0] = Xninit
vehicleN[1,0] = Yninit
start = time.time()

# time.sleep(10)  # or do something more productive

# position of vehicle n in each time step:
posNprev = 0
for i in range(rowVehN):
    for j in range(1,colVehN):
		if i == rowVehN - 1:
			vehicleN[i][j] = Yninit
		else:
			vehicleN[i][j] =  (Vninit * Ts) + Xninit # Xn = Vn*Ts + XnPrev!
			Xninit = vehicleN[i][j] # update the Xn based on previous timeStep
		
print "vehicle N :", vehicleN
# replicate = 5

done = time.time()
timneDiff = done - start
print "time spent = ", timneDiff

# with open("info.txt", "w") as out_file:
	# maxConstVio = "max constraint violation = "
	# maxConstVio += str(timeStep)
	# maxConstVio += "\n"
	# out_file.write(maxConstVio)

# with open("output_data" + str(replicate) + ".csv", "w") as out_file:
    # for i in range(len(fitness_list)):
        # out_string = ""
        # out_string += str(fitness_list[i])
        # out_string += "," + str(trait1_list[i])
        # out_string += "," + str(trait2_list[i])
        # out_string += "\n"
        # out_file.write(out_string)



 
# f = open('store.csv', 'wb')
# pickle.dump(vehicleN, f)
# f.close()
 
# f = open('store.csv', 'rb')
# obj = pickle.load(f)
# f.close()
		
		
# X = cvx.Variable(5*timeStep+3)
# P0 = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)
# q_obj = np.zeros((5*timeStep+3,1),dtype=float)

# q_c = np.zeros((5*timeStep+3,1),dtype=float)
# P_c = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)

# X = cvx.Variable(5*timeStep+3)

# # A = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)
# # b = np.zeros((5*timeStep+3,1),dtype=float)

# A = np.zeros((3*timeStep+3,5*timeStep+3),dtype=float)
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

# print "vehM = ", vehMInit




# velM_CD = np.zeros((2,timeStep + 1),dtype=float)
# timeStepVector = np.zeros((1,timeStep + 1),dtype=float)

# for j in range(timeStep+1):
	# timeStepVector[0][j] = j 


# for j in range(timeStep+1):
	# velM_CD[0][j] = 4*j+3	
	
# print "time step vector :", timeStepVector
# print "vel = ", velM_CD
# print "len Ts vec = ", len(timeStepVector[0])
# print "len vel = ", len(velM_CD[0])

# print "inv Ts = ", (1/Ts)*3

# plt.plot(timeStepVector[0],velM_CD[0])
# # plt.axis('tight')
# plt.show()
# # for t in range(timeStep):
	# # tmp = t+1
	# # P_c[5*tmp,5*tmp] = -2
	# # P_c[(5*tmp)+2,(5*tmp)+2] = -2

	# # q_c[5*tmp] = 2*vehicleN[0][t]
	# # q_c[(5*tmp)+2] = 2*vehicleN[1][t]
	# # r_c = (tmp)*(d**2) - (vehicleN[0][t])**2 - (vehicleN[1][t])**2

# # P_c = .5*(P_c+P_c.T)

# # print "P_c = ", P_c
# # print "qc = ", q_c
# # print "rc = ", r_c

# # for i in range(len(vehMInit)):
	# # b[i] = vehMInit[i]
	# # A[i,i] = 1

# # # print "b = ", b
# # # print "A = ", A

# # # rowA = len(vehMInit)
# # colA = 0
# # for rowA in range(len(vehMInit),3*timeStep+3,3):
	# # # print "Begin rowA =",rowA
	# # # for colA in range(5*timeStep+3):
	# # # print "Begin colA = ", colA
	# # # print"prob = ", A[rowA:rowA+3,colA:colA+3]
	# # A[rowA:rowA+3,colA:colA+3] = Amodel
	# # colA = colA+3
	# # # print "insert Amodel ..."
	# # # print "rowA =", rowA
	# # # print "colA =", colA
		
	# # A[rowA:rowA+3,colA:colA+2] = Bmodel
	# # colA = colA+2
	# # # print "insert Bmodel ..."
	# # # print "rowA =", rowA
	# # # print "colA =", colA
		
	# # A[rowA:rowA+3,colA:colA+3] = Cmodel
	# # # rowA = rowA+3
	
	# # # print "insert Cmodel ..."	
	# # # print "rowA =", rowA
	# # # print "colA =", colA
	# # # print "next loop"
	
		

# # # for i in range(len(vehMInit)-2,5*timeStep+3,5):
	# # # # print "filling out A matrix"
	# # # A[i,i] = 1
	# # # A[i+1,i+1] = 1

# # print "b = ", b
# # print "A = ", A

# # # newly modified code...

# # for i in range(timeStep):
	# # P0[5*(i+1)-2,5*(i+1)-2] = alpha
	# # P0[5*(i+1)-1,5*(i+1)-1] = beta
# # # print "P0 = ", P0
# # P_obj = .5*(P0+P0.T)
# # # print "P_obj = ", P_obj

# # q_obj[5*timeStep,0] = -1
# # # print "q_obj = ", q_obj


# # r_obj = Xminit
# # objective = cvx.Minimize(cvx.quad_form(X,P_obj) + q_obj.T*X  + r_obj)
