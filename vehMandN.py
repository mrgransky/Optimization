import matplotlib
matplotlib.use('Agg')


import cvxpy as cvx, numpy as np, matplotlib.pyplot as plt
# matplotlib.use('Agg')
from qcqp import *
import mosek
import time


timeStep = 17
Ts = .5 # sec !!! 

# laneLength = 2*eps + d
d = 2 # m
eps = .25 # m

alpha = .0001
beta = .01

# max & min Velocities for vehicle M
Vmax = 30 # m/s
Vmin = 0 # m/s

# max & min Y-pos for vehicle M
Ymax = 4 # m
Ymin = 1 # m

# max & min y-axis for vehicle M
aMax = 5 # m/s^2
aMin = -5 # m/s^2

rowVeh = 2
colVeh = timeStep +1  # initial pose + timeStep positions

timeStepVector = np.zeros((1,timeStep + 1),dtype=float)
for j in range(timeStep+1):
	timeStepVector[0][j] = j 

# Initialize matrices
vehicleN = np.zeros((rowVeh,colVeh),dtype=float)

# Position of vehicle M:
poseM_CD = np.zeros((rowVeh,colVeh),dtype=float)
poseM_DCCP = np.zeros((rowVeh,colVeh),dtype=float)
poseM_ADMM = np.zeros((rowVeh,colVeh),dtype=float)

# Velocity of vehicle M:
velM_CD = np.zeros((rowVeh,colVeh),dtype=float)
velM_DCCP = np.zeros((rowVeh,colVeh),dtype=float)
velM_ADMM = np.zeros((rowVeh,colVeh),dtype=float)


# Acceleration ( Longitudinal & Lateral ) of vehicle M:
accM_CD = np.zeros((rowVeh,timeStep),dtype=float)
accM_DCCP = np.zeros((rowVeh,timeStep),dtype=float)
accM_ADMM = np.zeros((rowVeh,timeStep),dtype=float)

tStepVecAcc = np.zeros((1,timeStep),dtype=float)
for idxj in range(timeStep):
	tStepVecAcc[0][idxj] = idxj 

X = cvx.Variable(5*timeStep+3)
P0 = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)
q_obj = np.zeros((5*timeStep+3,1),dtype=float)

# X = cvx.Variable(5*timeStep+3)

A = np.zeros((3*timeStep+3,5*timeStep+3),dtype=float)
b = np.zeros((3*timeStep+3,1),dtype=float)


# y_n = eps + d/2
# y_m = eps + d/2

# position of vehicle 'n':
Xninit, Vninit, Yninit = 7,20,(eps+d/2)
vehicleN[0,0] = Xninit
vehicleN[1,0] = Yninit

# vehicle 'm' Initial Condition:
Xminit, Vminit, Yminit , amInit, VymInit = 0,25,(eps+d/2),0,0

# Longitudinal Velocity V_x
velM_ADMM[0][0], velM_CD[0][0], velM_DCCP[0][0] = Vminit, Vminit, Vminit

# Lateral Velocity V_y
velM_ADMM[1][0], velM_CD[1][0], velM_DCCP[1][0] = VymInit, VymInit, VymInit

# Longitudinal Acceleration a_x !!!!!!!!!!!!!!!!!!??????????
accM_ADMM[0][0], accM_CD[0][0], accM_DCCP[0][0] = amInit, amInit, amInit

# Lateral Acceleration a_y (not Interested!!)
accM_ADMM[1][0], accM_CD[1][0], accM_DCCP[1][0] = 0,0,0 # Zero lateral acceleration 

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
			posNprev = vehicleN[i][j] # update the Xn based on previous timeStep
		
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



for i in range(timeStep):
	P0[5*(i+1)-2,5*(i+1)-2] = alpha
	P0[5*(i+1)-1,5*(i+1)-1] = beta
P_obj = .5*(P0+P0.T)
# print "P_obj = ", P_obj

q_obj[5*timeStep,0] = -1
# print "q_obj = ", q_obj

r_obj = Xminit
	
objective = cvx.Minimize(cvx.quad_form(X,P_obj) + q_obj.T*X  + r_obj)
# -------------------------------------------------------------------

# Linear Constraint A.X = b
for i in range(len(vehMInit)): # needs to be modified!!!!
	b[i] = vehMInit[i]
	A[i,i] = 1

colA = 0
# for rowA in range(len(vehMInit),5*timeStep+3,5):

for rowA in range(len(vehMInit),3*timeStep+3,3):
	A[rowA:rowA+3,colA:colA+3] = Amodel
	colA = colA+3
	A[rowA:rowA+3,colA:colA+2] = Bmodel
	colA = colA+2
	A[rowA:rowA+3,colA:colA+3] = Cmodel
	
consTot = []	

# Constraint for Velocity
for j in range(timeStep):
	tmpIdx = j+1
	consTot.append(X[(5*tmpIdx)+1] <= Vmax)
	consTot.append(X[(5*tmpIdx)+1] >= Vmin)

# Constraint for Y-axis	
for j in range(timeStep):
	tmpIdx = j+1
	consTot.append(X[(5*tmpIdx)+2] <= Ymax)
	consTot.append(X[(5*tmpIdx)+2] >= Ymin)


for j in range(timeStep):
	tmpIdx = j
	consTot.append(X[(5*tmpIdx)+3] <= aMax)
	consTot.append(X[(5*tmpIdx)+3] >= aMin)

	

# print "b = ", b
# print "A = ", A
consLin = A*X == b
# ----------------------------------------------------

# Quadratic Constraint:
for t in range(timeStep):
	q_c = np.zeros((5*timeStep+3,1),dtype=float)
	P_c = np.zeros((5*timeStep+3,5*timeStep+3),dtype=float)
	
	tmp = t+1 # initial state is excluded from constraints while last timeStep is included!
	P_c[5*tmp,5*tmp] = -2
	P_c[(5*tmp)+2,(5*tmp)+2] = -2

	q_c[5*tmp] = 2*vehicleN[0][tmp]
	q_c[(5*tmp)+2] = 2*vehicleN[1][tmp]
	r_c = d**2 - (vehicleN[0][tmp])**2 - (vehicleN[1][tmp])**2
	P_c = .5*(P_c+P_c.T)
	consTot.append(.5*cvx.quad_form(X,P_c) + q_c.T*X + r_c <= 0)


# print "P_c = ", P_c
# print "qc = ", q_c
# print "rc = ", r_c


consTot.append(consLin)

# consTot = [consLin]
	
## ------------------- formulating the problem: ------------------- ##
tProbSt = time.time()

prob = cvx.Problem(objective, consTot)
qcqp = QCQP(prob)

tProbEn = time.time()
tDiffProb = tProbEn - tProbSt
print "Problem formulation time [sec] = ", tDiffProb


print "----------------- Program is starting ----------------"

# qcqp.suggest(SPECTRAL)
# print("Spectral-based lower bound: %.3f" % qcqp.spectral_bound)

# # # # ----------------------- Coordinate Decent (CD): -------------------------

# # suggest method : SDR
# qcqp.suggest(SDR, solver=cvx.MOSEK)
# sdrSug = qcqp.sdr_bound
# print("SDR-based lower bound: %.3f" % sdrSug)

# # Attempt to improve the starting point given by the suggest method
# f_cd, v_cd = qcqp.improve(COORD_DESCENT)
# print("CD: objective value = %.3f, max constraint violation = %.3f" % (f_cd, v_cd))

# xCD = np.copy(X.value)
# print "xCD = ", xCD

# # Vehicle M Position, CD improve:
# for iM in range(timeStep+1):
	# poseM_CD[0][iM] = xCD[5*iM]
	# poseM_CD[1][iM] = xCD[(5*iM)+2]


# # Vehicle M Velocity, CD improve:
# for iM in range(timeStep+1):
	# velM_CD[0][iM] = xCD[(5*iM)+1]
	# # velM_ADMM[1][iM] = xADMM[(5*iM)+4]

# # Vehicle M Acceleration, CD improve:
# for iM in range(timeStep):
	# accM_CD[0][iM+1] = xCD[(5*iM)+3]

	
# print "VehN: ", vehicleN
# print "poseM_CD: ", poseM_CD
# print "VelM_CD: ", velM_CD
# print "accM_CD: ", accM_CD

# ----------------------- Coordinate Decent (ADMM): -------------------------

tStSugADMM = time.time()
# suggest method : SDR for ADMM
qcqp.suggest(SDR, solver=cvx.MOSEK)
sdrSugADMM = qcqp.sdr_bound
tEnSugADMM = time.time()
tDiffSugADMM = tEnSugADMM - tStSugADMM
print("ADMM; SDR-based lower bound = %.3f, duration [sec] = %.5f " % (sdrSugADMM, tDiffSugADMM))

# Attempt to improve the starting point given by the suggest method
tStImpADMM = time.time()

f_ADMM, v_ADMM = qcqp.improve(ADMM)

tEnImpADMM = time.time()

tDiffImpADMM = tEnImpADMM - tStImpADMM
print("ADMM: objective value = %.3f, max constraint violation = %.3f , duration [sec] %.5f = " % (f_ADMM, v_ADMM, tDiffImpADMM))

xADMM = np.copy(X.value)	
print "xADMM = ", xADMM


# Vehicle M Position, ADMM improve:
for iM in range(timeStep+1):
	poseM_ADMM[0][iM] = xADMM[5*iM]
	poseM_ADMM[1][iM] = xADMM[(5*iM)+2]

# Vehicle M Velocity, ADMM improve:
for iM in range(timeStep+1):
	velM_ADMM[0][iM] = xADMM[(5*iM)+1]

# Vehicle M Acceleration, ADMM improve:
for iM in range(timeStep):
	accM_ADMM[0][iM] = xADMM[(5*iM)+3]


print "Vehicle N: ", vehicleN
print "poseM_ADMM: ", poseM_ADMM
print "VelM_ADMM: ", velM_ADMM
print "accM_ADMM: ", accM_ADMM

# ----------------------- Coordinate Decent (DCCP): -------------------------
tStSugDCCP = time.time()
# suggest method : SDR DCCP
qcqp.suggest(SDR, solver=cvx.MOSEK)
sdrSugDCCP = qcqp.sdr_bound
tEnSugDCCP = time.time()
tDiffSugDCCP = tEnSugADMM - tStSugADMM
print("DCCP; SDR-based lower bound = %.3f , duration [sec] = %.5f " % (sdrSugDCCP, tDiffSugDCCP))

# Attempt to improve the starting point given by the suggest method
tStImpDCCP = time.time()
f_DCCP, v_DCCP = qcqp.improve(DCCP)
tEnImpDCCP = time.time()
tDiffImpDCCP = tEnImpDCCP - tStImpDCCP 
print("DCCP: objective value = %.3f, max constraint violation = %.3f , duration [sec] = %.5f " % (f_DCCP, v_DCCP, tDiffImpDCCP))
xDCCP = np.copy(X.value)
print "xDCCP = ", xDCCP


# Vehicle M Position, DCCP improve:
for iM in range(timeStep+1):
	poseM_DCCP[0][iM] = xDCCP[5*iM]
	poseM_DCCP[1][iM] = xDCCP[(5*iM)+2]
	

# Vehicle M Velocity, DCCP improve:
for iM in range(timeStep+1):
	velM_DCCP[0][iM] = xDCCP[(5*iM)+1]
	
# Vehicle M Acceleration, DCCP improve:
for iM in range(timeStep):
	accM_DCCP[0][iM] = xDCCP[(5*iM)+3]

print "Vehicle N: ", vehicleN
print "poseM_DCCP: ", poseM_DCCP
print "VelM_DCCP: ", velM_DCCP
print "accM_DCCP: ", accM_DCCP

with open("readMe.txt", "w") as out_file:
	ts = "Ts = "
	ts += str(Ts)
	ts += " \n \n"
	
	tStp = " timeStep = "
	tStp += str(timeStep)
	tStp += "\n \n"
	
	diameter = " d = "
	diameter += str(d)
	diameter += " \n \n"
	
	timeSuggestADMM = " ADMM suggest duration [sec] = "
	timeSuggestADMM += str(tDiffSugADMM)
	timeSuggestADMM += "\n\n"
	
	lwBoundADMM = " ADMM suggest SDR lower bound = "
	lwBoundADMM += str(sdrSugADMM)
	lwBoundADMM += "\n \n"
	
	objValADMM = " ADMM: objective value = "
	objValADMM += str(f_ADMM)
	objValADMM += " \n \n"
	
	maxVioADMM = "ADMM: max constraint violation = "
	maxVioADMM += str(v_ADMM)
	maxVioADMM += " \n \n"
	
	timeSuggestDCCP = " DCCP suggest duration [sec] = "
	timeSuggestDCCP += str(tDiffSugDCCP)
	timeSuggestDCCP += " \n \n"
	
	lwBoundDCCP = " DCCP suggest SDR lower bound = "
	lwBoundDCCP += str(sdrSugDCCP)
	lwBoundDCCP += " \n \n"
	
	objValDCCP = " DCCP: objective value = "
	objValDCCP += str(f_DCCP)
	objValDCCP += " \n \n"
	
	maxVioDCCP = " DCCP max constraint violation = "
	maxVioDCCP += str(v_DCCP)
	maxVioDCCP += " \n \n"
	
	out_file.write(ts)
	out_file.write(tStp)
	out_file.write(diameter)
	
	out_file.write(timeSuggestADMM)
	out_file.write(lwBoundADMM)
	out_file.write(objValADMM)
	out_file.write(maxVioADMM)
	
	out_file.write(timeSuggestDCCP)
	out_file.write(lwBoundDCCP)
	out_file.write(objValDCCP)
	out_file.write(maxVioDCCP)
	

# --------------------------- Plotting ------------------------------
circ = np.linspace(0, 2*np.pi)


# # CD Trajectory:
# plt.figure()
# for idx in range(timeStep+1):
	# if idx == 0: # initial positions for vehicle 'm' & 'n':
		# plt.plot(xCD[idx]+(d/2)*np.cos(circ),xCD[idx+2]+(d/2)*np.sin(circ), 'c')
		# plt.plot(vehicleN[0][idx]+(d/2)*np.cos(circ),vehicleN[1][idx]+(d/2)*np.sin(circ), 'm')
	# else:
		# tmp = idx
		# plt.plot(xCD[5*tmp]+(d/2)*np.cos(circ),xCD[(5*tmp)+2]+(d/2)*np.sin(circ), 'b')
		# plt.plot(vehicleN[0][tmp]+(d/2)*np.cos(circ),vehicleN[1][tmp]+(d/2)*np.sin(circ), 'r')
		
# plt.grid()
# plt.axis([-2, 34, 0, 35])
# plt.xlabel('X [m]')
# plt.ylabel('Y [m]')
# plt.title('CD Trajectory')
# plt.legend(["Initial Vehicle m","Initial Vehicle n","Vehicle m","Vehicle n"])

# # CD y vs timeStep
# plt.figure()
# plt.plot(timeStepVector[0],poseM_CD[1],'.g-')
# plt.grid()
# # plt.axis('equal')
# plt.xlabel('time step')
# plt.ylabel('y [m]')
# plt.title('CD Y-axis')

# # CD Velocity:
# plt.figure()
# plt.plot(timeStepVector[0],velM_CD[0],'.r-')
# plt.grid()
# # plt.axis('equal')
# plt.xlabel('time step')
# plt.ylabel('V_{x} [m/s]')
# plt.title('CD Velocity')

# # CD Acceleration:
# plt.figure()
# plt.plot(timeStepVector[0],accM_CD[0],'xb-')
# plt.grid()
# # plt.axis('equal')
# plt.xlabel('time step')
# plt.ylabel('a [m/s^2]')
# plt.title('CD Acceleration')

# ADMM Trajectory:
plt.figure()
for idx in range(timeStep+1):
	if idx == 0:
		plt.plot(xADMM[idx]+(d/2)*np.cos(circ),xADMM[idx+2]+(d/2)*np.sin(circ), 'c')
		plt.plot(vehicleN[0][idx]+(d/2)*np.cos(circ),vehicleN[1][idx]+(d/2)*np.sin(circ), 'm')
	else:
		tmp = idx
		plt.plot(xADMM[5*tmp]+(d/2)*np.cos(circ),xADMM[(5*tmp)+2]+(d/2)*np.sin(circ), 'b')
		plt.plot(vehicleN[0][tmp]+(d/	2)*np.cos(circ),vehicleN[1][tmp]+(d/2)*np.sin(circ), 'r')
		
plt.grid()
# plt.axis([-2, 200, Ymin-d/2, Ymax+d/2])
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('ADMM Trajectory')
plt.legend(["Initial Vehicle m","Initial Vehicle n","Vehicle m","Vehicle n"])
plt.savefig('ADMMtraj.jpeg', dpi = 500)

# ADMM y vs timeStep
plt.figure()
plt.plot(timeStepVector[0],poseM_ADMM[1],'.g-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('y [m]')
plt.title('ADMM Y-axis')
plt.savefig('ADMMy-axis.jpeg', dpi = 500)

# ADMM Velocity:
plt.figure()
plt.plot(timeStepVector[0],velM_ADMM[0],'.r-')
plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('V_{x} [m/s]')
plt.title('ADMM Velocity')
plt.savefig('ADMMvel.jpeg', dpi = 500)

# ADMM Acceleration:
plt.figure()
plt.plot(tStepVecAcc[0],accM_ADMM[0],'xb-')

plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('a [m/s^2]')
plt.title('ADMM Acceleration')
plt.savefig('ADMMacc.jpeg', dpi = 500)


# DCCP Trajectory:
plt.figure()
for idx in range(timeStep+1):
	if idx == 0:
		plt.plot(xDCCP[idx]+(d/2)*np.cos(circ),xDCCP[idx+2]+(d/2)*np.sin(circ), 'c')
		plt.plot(vehicleN[0][idx]+(d/2)*np.cos(circ),vehicleN[1][idx]+(d/2)*np.sin(circ), 'm')
	else:
		tmp = idx
		plt.plot(xDCCP[5*tmp]+(d/2)*np.cos(circ),xDCCP[(5*tmp)+2]+(d/2)*np.sin(circ), 'b')
		plt.plot(vehicleN[0][tmp]+(d/2)*np.cos(circ),vehicleN[1][tmp]+(d/2)*np.sin(circ), 'r')
		
plt.grid()
# plt.axis([-2, 200, Ymin-d/2, Ymax+d/2])
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('DCCP Trajectory')
plt.legend(["Initial Vehicle m","Initial Vehicle n","Vehicle m","Vehicle n"])
plt.savefig('DCCPtraj.jpeg', dpi = 500)

# DCCP y vs timeStep
plt.figure()
plt.plot(timeStepVector[0],poseM_DCCP[1],'.g-')

plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('y [m]')
plt.title('DCCP Y-axis')
plt.savefig('DCCPy-axis.jpeg', dpi = 500)


# DCCP Velocity:
plt.figure()
plt.plot(timeStepVector[0],velM_DCCP[0],'.r-')

plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('V_{x} [m/s]')
plt.title('DCCP Velocity')
plt.savefig('DCCPvel.jpeg', dpi = 500)

# DCCP Acceleration:
plt.figure()
plt.plot(tStepVecAcc[0],accM_DCCP[0],'xb-')

plt.grid()
# plt.axis('equal')
plt.xlabel('time step')
plt.ylabel('a [m/s^2]')

plt.title('DCCP Acceleration')
plt.savefig('DCCPacc.jpeg', dpi = 500)

plt.show()