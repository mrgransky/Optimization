from __future__ import division
import matplotlib
matplotlib.use('Agg')

import cvxpy as cvx, numpy as np, matplotlib.pyplot as plt
from qcqp import *
import mosek
import time

# ------------------------------- #
# Simulation time [sec] = K * Ts 
simTime = 4 # sec
Ts = 1 # sec !!!
K = simTime/Ts

print "K = " , K

# ------------------------------- #
# laneLength = 2*eps + d
d = 3 # m
eps = .25 # m

alpha = 55
beta = 3

# max & min Velocities for vehicle M
Vmax = 30 # m/s
Vmin = 16 # m/s

# max & min Y-pos for vehicle M
Ymax = 2*d +4*eps # m
Ymin = eps + d/2 # m

# max & min y-axis for vehicle M
aMax = 5 # m/s^2
aMin = -5 # m/s^2

rowVeh = 2
colVeh = K +1  # initial pose + K positions

KVector = np.zeros((1,K + 1),dtype=float)
for j in range(K+1):
	KVector[0][j] = j

# Initialize matrices
vehicleN = np.zeros((rowVeh,colVeh),dtype=float)

# Position of vehicle M:
poseM_CD = np.zeros((rowVeh,colVeh),dtype=float)
poseM_DCCP = np.zeros((rowVeh,colVeh),dtype=float)
poseM_ADMM = np.zeros((rowVeh,colVeh),dtype=float)

# Velocity of vehicle M:
velM_CD = np.zeros((rowVeh,colVeh),dtype=float)
velM_DCCP = np.zeros((rowVeh,colVeh),dtype=float)
# velM_ADMM = np.zeros((rowVeh,colVeh),dtype=float)
velM_ADMMLong = np.zeros((1,K+1),dtype=float)
velM_ADMMLat = np.zeros((1,K),dtype=float)


# Acceleration ( Longitudinal & Lateral ) of vehicle M:
accM_CD = np.zeros((rowVeh,K),dtype=float)
accM_DCCP = np.zeros((rowVeh,K),dtype=float)
accM_ADMM = np.zeros((1,K),dtype=float)

tStpVec = np.zeros((1,K),dtype=float)
for idxj in range(K):
	tStpVec[0][idxj] = idxj 

X = cvx.Variable(5*K+3)
P0 = np.zeros((5*K+3,5*K+3),dtype=float)
q_obj = np.zeros((5*K+3,1),dtype=float)

A = np.zeros((3*K+3,5*K+3),dtype=float)
b = np.zeros((3*K+3,1),dtype=float)


# y_n = eps + d/2
# y_m = eps + d/2

# position of vehicle 'n':
Xninit, Vninit, Yninit = 20,Vmin,(eps + d/2)
vehicleN[0][0] = Xninit
vehicleN[1][0] = Yninit

# vehicle 'm' Initial Condition:
Xminit, Vminit, Yminit , amInit, VymInit = 0,Vmin,(eps+d/2),0,0

# Longitudinal Velocity V_x
# velM_ADMM[0][0], velM_CD[0][0], velM_DCCP[0][0] = Vminit, Vminit, Vminit

# Lateral Velocity V_y
velM_ADMMLat[0][0], velM_CD[1][0], velM_DCCP[1][0] = VymInit, VymInit, VymInit

# Longitudinal Acceleration a_x !!!!!!!!!!!!!!!!!!??????????
accM_ADMM[0][0], accM_CD[0][0], accM_DCCP[0][0] = amInit, amInit, amInit

# Lateral Acceleration a_y (not Interested!!)
accM_ADMM[0][0], accM_CD[1][0], accM_DCCP[1][0] = 0,0,0 # Zero lateral acceleration 

vehMInit = ([
		[Xminit],[Vminit],[Yminit]
		])

# position of vehicle n in each time step:
posNprev = Xninit

for i in range(rowVeh):
    for j in range(1,colVeh):
		if i == rowVeh - 1:
			vehicleN[i][j] = Yninit
		else:
			vehicleN[i][j] =  (Vninit * Ts) + posNprev # Xn = Vn*Ts + XnPrev!
			posNprev = vehicleN[i][j] # update the Xn based on previous K
		
# objective function: 
# System model:
Amodel = np.matrix([[-1,-Ts,0],
					[0,-1,0],
					[0,0,-1]
					])
Bmodel = np.matrix([[-.5*(Ts**2),0],
					[-Ts,0],
					[0,-Ts]
					])
Cmodel = np.identity(3)


for i in range(K):
	P0[5*(i+1)-2,5*(i+1)-2] = alpha
	P0[5*(i+1)-1,5*(i+1)-1] = beta
# P_obj = .5*(P0+P0.T)
P_obj = P0
# print "P_obj = ", P_obj

q_obj[5*K,0] = -1
# print "q_obj = ", q_obj

r_obj = Xminit
	
objective = cvx.Minimize(cvx.quad_form(X,P_obj) + q_obj.T*X  + r_obj)
# -------------------------------------------------------------------	
# -------------------------- Constraints --------------------------
consTot = []

# Linear Constraint A.X = b
for i in range(len(vehMInit)): # needs to be modified!!!!
	b[i] = vehMInit[i]
	A[i,i] = 1

colA = 0
for rowA in range(len(vehMInit),3*K+3,3):
	A[rowA:rowA+3,colA:colA+3] = Amodel
	colA = colA+3
	A[rowA:rowA+3,colA:colA+2] = Bmodel
	colA = colA+2
	A[rowA:rowA+3,colA:colA+3] = Cmodel
	
consTot.append(A*X == b)

# Constraint for Velocity
for j in range(K):
	tmpIdx = j+1
	consTot.append(X[(5*tmpIdx)+1] <= Vmax)
	consTot.append(X[(5*tmpIdx)+1] >= Vmin)

# Constraint for Y-axis	
for j in range(K):
	tmpIdx = j+1
	consTot.append(X[(5*tmpIdx)+2] <= Ymax)
	consTot.append(X[(5*tmpIdx)+2] >= Ymin)

for j in range(K):
	tmpIdx = j
	consTot.append(X[(5*tmpIdx)+3] <= aMax)
	consTot.append(X[(5*tmpIdx)+3] >= aMin)

# -------------- Quadratic Constraint -----------------
prev_rc = 0
# Quadratic Constraint:
for t in range(K):
	q_c = np.zeros((5*K+3,1),dtype=float)
	P_c = np.zeros((5*K+3,5*K+3),dtype=float)
	tmp = t+1 # initial state is excluded from constraints while last K is included!
	P_c[5*tmp,5*tmp] = -2
	P_c[(5*tmp)+2,(5*tmp)+2] = -2

	q_c[5*tmp] = 2*vehicleN[0][tmp]
	q_c[(5*tmp)+2] = 2*vehicleN[1][tmp]
	r_c = d**2 - (vehicleN[0][tmp])**2 - (vehicleN[1][tmp])**2 + prev_rc
	print "rc[", tmp, "] = ",r_c
	# prev_rc = r_c
	# P_c = .5*(P_c+P_c.T)
	consTot.append(cvx.quad_form(X,P_c) + q_c.T*X + r_c <= 0)


# print "P_c = ", P_c
# print "qc = ", q_c
# print "rc = ", r_c
	
## ------------------- formulating the problem: ------------------- ##
tProbSt = time.time()

prob = cvx.Problem(objective, consTot)
qcqp = QCQP(prob)
tProbEn = time.time()
tDiffProb = tProbEn - tProbSt

print "Problem formulation time [sec] = ", tDiffProb

# # ----------- alternating directions method of multipliers (ADMM): -----------
# ------------ Suggest; ADMM ------------
tStSugADMM = time.time()

qcqp.suggest(SDR, solver=cvx.MOSEK)

sdrSugADMM = qcqp.sdr_bound
sdrSugSol = qcqp.sdr_sol

tEnSugADMM = time.time()

tDiffSugADMM = tEnSugADMM - tStSugADMM

xSugADMM = np.copy(X.value)
# print "xADMM Suggest = ", xSugADMM

# print "sdr_sol = ", sdrSugSol
# print "ttteemmp = ", sdrSugSol[:-1, -1]

print("ADMM; SDR-based lower bound = %.3f, duration [sec] = %.5f " % (sdrSugADMM, tDiffSugADMM))

# ------------ Improve ADMM: ------------
tStImpADMM = time.time()

f_ADMM, v_ADMM = qcqp.improve(ADMM)

tEnImpADMM = time.time()

tDiffImpADMM = tEnImpADMM - tStImpADMM
print("ADMM: objective value = %.3f, max constraint violation = %.3f , duration [sec] %.5f = " % (f_ADMM, v_ADMM, tDiffImpADMM))

xADMM = np.copy(X.value)	
print "xADMM = ", xADMM


# Vehicle M Position, ADMM improve:
for iM in range(K+1):
	poseM_ADMM[0][iM] = xADMM[5*iM]
	poseM_ADMM[1][iM] = xADMM[(5*iM)+2]

# Vehicle M Velocity, ADMM improve:
for iM in range(K+1):
	velM_ADMMLong[0][iM] = xADMM[(5*iM)+1]

for iMvel in range(K):
	velM_ADMMLat[0][iMvel] = xADMM[5*iMvel+4]
	
# Vehicle M Acceleration, ADMM improve:
for iM in range(K):
	accM_ADMM[0][iM] = xADMM[(5*iM)+3]


print "Vehicle N: ", vehicleN
print "poseM_ADMM: ", poseM_ADMM
print "VelM_ADMM Longitudinal = ", velM_ADMMLong
print "VelM_ADMM Lateral = ", velM_ADMMLat
print "accM_ADMM: ", accM_ADMM

# # ----------------------- (DCCP): -------------------------
# # ----------------------------------------

with open("SysSpec.txt", "w") as out_file:
	ts = "Ts = "
	ts += str(Ts)
	ts += " \n \n"
	
	tStp = " K = "
	tStp += str(K)
	tStp += " \n \n"
	
	diameter = " d = "
	diameter += str(d)
	diameter += " \n \n"
	
	_alfa = " Alpha = "
	_alfa += str(alpha)
	_alfa += " \n \n"
	
	_beta = " Beta = "
	_beta += str(beta)
	_beta += " \n \n"
	
	xN = "Initial XPose Vehicle N = "
	xN += str(Xninit)
	xN += " \n \n "
	
	xM = "Initial XPose Vehicle M = "
	xM += str(Xminit)
	xM += " \n \n "
	
	velMinit = "Vm_init = "
	velMinit += str(Vminit)
	velMinit += " \n \n "
	
	velNinit = "VnInit = "
	velNinit += str(Vninit)
	velNinit += " \n \n "
	
	
	timeSuggestADMM = " ADMM suggest duration [sec] = "
	timeSuggestADMM += str(tDiffSugADMM)
	timeSuggestADMM += "\n\n"
	
	lwBoundADMM = " ADMM suggest SDR lower bound = "
	lwBoundADMM += str(sdrSugADMM)
	lwBoundADMM += "\n \n"
	
	tImpADMM = " ADMM improve duration [sec] = "
	tImpADMM += str(tDiffImpADMM)
	tImpADMM += "\n \n"
	
	objValADMM = " ADMM: objective value = "
	objValADMM += str(f_ADMM)
	objValADMM += " \n \n"
	
	maxVioADMM = " ADMM: max constraint violation = "
	maxVioADMM += str(v_ADMM)
	maxVioADMM += " \n \n"
	
	out_file.write(ts)
	out_file.write(tStp)
	out_file.write(diameter)

	out_file.write(_alfa)
	out_file.write(_beta)
	
	out_file.write(velMinit)
	out_file.write(velNinit)
	
	out_file.write(xM)
	out_file.write(xN)
	
	
	out_file.write(timeSuggestADMM)
	out_file.write(lwBoundADMM)
	out_file.write(tImpADMM)
	out_file.write(objValADMM)
	out_file.write(maxVioADMM)
	
	# out_file.write(timeSuggestDCCP)
	# out_file.write(lwBoundDCCP)
	# out_file.write(tImpDCCP)
	# out_file.write(objValDCCP)
	# out_file.write(maxVioDCCP)
	

# --------------------------- Plotting ------------------------------
circ = np.linspace(0, 2*np.pi)

# Trajectory, Suggest:
plt.figure()
for idx in range(K+1):
	if idx == 0:
		plt.plot(xSugADMM[idx]+(d/2)*np.cos(circ),xSugADMM[idx+2]+(d/2)*np.sin(circ), 'c')
		plt.plot(vehicleN[0][idx]+(d/2)*np.cos(circ),vehicleN[1][idx]+(d/2)*np.sin(circ), 'm')
	else:
		tmp = idx
		plt.plot(xSugADMM[5*tmp]+(d/2)*np.cos(circ),xSugADMM[(5*tmp)+2]+(d/2)*np.sin(circ), 'b')
		plt.plot(vehicleN[0][tmp]+(d/2)*np.cos(circ),vehicleN[1][tmp]+(d/2)*np.sin(circ), 'r')
		
plt.grid()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Trajectory based on Suggest')
plt.savefig('trajSuggest.jpeg', dpi = 600)

# ADMM Path, Improve:
plt.figure()
for idx in range(K+1):
	if idx == 0:
		plt.plot(poseM_ADMM[0][idx],poseM_ADMM[1][idx], 'oc')
		plt.plot(vehicleN[0][idx],vehicleN[1][idx], 'om')
	else:
		# tmp = idx
		plt.plot(poseM_ADMM[0][idx],poseM_ADMM[1][idx],'xb')
		plt.plot(vehicleN[0][idx],vehicleN[1][idx], 'xr')
		
plt.grid()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Path')
plt.savefig('ADMMPath.jpeg', dpi = 600)




# ADMM Trajectory, Improve:
plt.figure()
for idx in range(K+1):
	if idx == 0:
		plt.plot(xADMM[idx]+(d/2)*np.cos(circ),xADMM[idx+2]+(d/2)*np.sin(circ), 'c')
		plt.plot(vehicleN[0][idx]+(d/2)*np.cos(circ),vehicleN[1][idx]+(d/2)*np.sin(circ), 'm')
	else:
		tmp = idx
		plt.plot(xADMM[5*tmp]+(d/2)*np.cos(circ),xADMM[(5*tmp)+2]+(d/2)*np.sin(circ), 'b')
		plt.plot(vehicleN[0][tmp]+(d/2)*np.cos(circ),vehicleN[1][tmp]+(d/2)*np.sin(circ), 'r')
		
plt.grid()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Trajectory based on ADMM Improve')
plt.savefig('ADMMtraj.jpeg', dpi = 600)

# ADMM y vs K
plt.figure()
plt.plot(KVector[0],poseM_ADMM[1],'.y-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step (k)')
plt.ylabel('y [m]')
plt.title('ADMM Y-axis')
plt.savefig('ADMMy-axis.jpeg', dpi = 600)

# ADMM x-K
plt.figure()
plt.plot(KVector[0],poseM_ADMM[0],'.g-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step (k)')
plt.ylabel('x [m]')
plt.title('ADMM X-axis')
plt.savefig('ADMMx-axis.jpeg', dpi = 600)



# ADMM Longitudinal Velocity:
plt.figure()
plt.plot(KVector[0],velM_ADMMLong[0],'.r-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('V_{x} [m/s]')
plt.title('ADMM Longitudinal Velocity')
plt.savefig('ADMMvelLong.jpeg', dpi = 600)


# print "tStp = ", tStpVec

# ADMM Lateral Velocity:
plt.figure()
plt.plot(tStpVec[0],velM_ADMMLat[0],'.k-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step (k)')
plt.ylabel('V_{y} [m/s]')
plt.title('ADMM Lateral Velocity')
plt.savefig('ADMMvelLat.jpeg', dpi = 600)

# ADMM Acceleration:
plt.figure()
plt.plot(tStpVec[0],accM_ADMM[0],'xb-')

plt.grid()
# plt.axis('equal')
plt.xlabel('time step (k)')
plt.ylabel('a [m/s^2]')
plt.title('ADMM Acceleration')
plt.savefig('ADMMacc.jpeg', dpi = 600)

# # DCCP Trajectory:
# -----------------------------------------
plt.show()