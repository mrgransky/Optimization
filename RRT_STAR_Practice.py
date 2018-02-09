"""
Path Planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)

"""
from __future__ import division
import matplotlib
matplotlib.use('Agg')

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time
show_animation = True
import os, datetime

# -------------------------------------------------------------------------------------------------
# Four different scenarios:
# 1. Vf = 10, Vm = 10, expectation: straight line (almost OK!!!!)
# 2. Vf = 10, Vm = 5, expectation: overtake (almost OK!!!!)
# 3. Vf = 5, Vm = 10, expectation: straight line (Decent!!!)
# 4. Static Goal : Vf = 12, Vm = 10, Pf = (0,2), Pm = (16,2), G(84,2) => expectation: straight line
# -------------------------------------------------------------------------------------------------





# vehicle Follower(F)
Vf = 10
V_m = 5
Ts = 1

K = 0
WIDTH_FREEWAY = 12 
LENGTH_FREEWAY = 500
NUM_LANES = 3
LANE_LENGH = WIDTH_FREEWAY / NUM_LANES

results_dir = os.path.join(os.getcwd(), 'Results/Feb8/')
sample_file_name = datetime.datetime.now().strftime('%m%d_%H%M') + "_vF="+str(Vf) + "_vM="+str(V_m) + "_.jpeg"

t = np.linspace(0, 2*np.pi,1000)

    
if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

class RRTSTAR():
	def __init__(self, start, goal, obstacleList, randArea,expandDis=1, goalSampleRate=99, maxIter=1000):
		self.start = Node(start[0], start[1])
		self.end = Node(goal[0], goal[1])
		
		self.minRndX = randArea[0]
		self.maxRndX = randArea[1]
		
		self.minRndY = randArea[2]
		self.maxRndY = randArea[3]
		
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList
		
	def Planning(self, animation=True):
			self.nodeList = [self.start]
			for i in range(self.maxIter):
					rnd = self.get_random_point()
					[minDisIdx,minTimeIdx] = self.getClosestNodeIdx(self.nodeList, rnd)
					
					# print "min Distance Index = ", minDisIdx,", min Time Index = ", minTimeIdx, "\n"
					# print "Node [",minDisIdx,"] = (",self.nodeList[minDisIdx].x,",",self.nodeList[minDisIdx].y,"), Node [",minTimeIdx,"]=(",self.nodeList[minTimeIdx].x,",",self.nodeList[minTimeIdx].y,")\n\n"
					
					# now it only works for minimum distance 
					selNode = self.steer(rnd, minDisIdx,minTimeIdx)
					if self.__CollisionCheck(selNode, self.obstacleList):
					
						# draw a circle around a selected node to find the nearest nodes
						nearinds = self.find_near_nodes(selNode)
						
						newNode = self.choose_parent(selNode, nearinds)
						
						self.nodeList.append(newNode)
						self.rewire(newNode, nearinds)
					# newNode2Point = [newNode.x,newNode.y]
					if animation:
						self.DrawGraph(rnd)
			lastIndex = self.get_best_last_index()
			path = self.gen_final_course(lastIndex)
			return path
			
	def choose_parent(self, givenNode, nearinds):
		if len(nearinds) == 0:
			return givenNode
		costList = []
		for i in nearinds:
			dx = givenNode.x - self.nodeList[i].x
			dy = givenNode.y - self.nodeList[i].y
			d = math.sqrt(dx ** 2 + dy ** 2)
			theta = math.atan2(dy, dx)
			tTime = d/Vf
			if self.check_collision_extend(self.nodeList[i], theta, d):
				costList.append([self.nodeList[i].cost + d,self.nodeList[i].time+ tTime])
			else:
				costList.append([float("inf"),0])
		# print "Cost = ", costList
		
		
		mincost = float("inf")
		for i in range(len(costList)):
			if costList[i][0] < mincost :
				mincost = costList[i][0]
				minTime = costList[i][1]
				minCostNodeIdx = nearinds[i]
		
		
		if mincost == float("inf"):
			print("mincost is inf")
			return givenNode
		
		givenNode.cost = mincost
		givenNode.time = minTime
		givenNode.parent = minCostNodeIdx
		# print "cost = ", mincost, "time = ", minTime, "temp idx = ", minCostNodeIdx 
		
		# print "newNode_2 = (", givenNode.x, ",", givenNode.y,") , Cost = ", givenNode.cost, " , parent = ", givenNode.parent, "\n\n"
		return givenNode
	
	def steer(self, rnd, inpMinDisNodeIdx, inpMinTimeNodeIdx):
		nearestNodeDis = self.nodeList[inpMinDisNodeIdx]
		nearestNodeTime = self.nodeList[inpMinTimeNodeIdx]
		# slope
		theta = math.atan2(	rnd[1] - nearestNodeDis.y, rnd[0] - nearestNodeDis.x)
		
		newNode = copy.deepcopy(nearestNodeDis)
		
		newNode.x += self.expandDis * math.cos(theta)
		newNode.y += self.expandDis * math.sin(theta)
		
		newNode.cost += self.expandDis
		newNode.time += abs(newNode.x - nearestNodeDis.x)/Vf
		newNode.parent = inpMinDisNodeIdx
		# print "newNode = (", newNode.x, ",", newNode.y,") , Cost = ", newNode.cost, " , parent = ", newNode.parent, "\n"
		plt.plot(newNode.x,newNode.y,"yx")
					
		return newNode
	
	def get_random_point(self):
		if random.randint(0, 100) > self.goalSampleRate:
			rnd = [random.uniform(self.minRndX, self.maxRndX),random.uniform(self.minRndY ,self.maxRndY)]
		else:  # goal point sampling
			rnd = [self.end.x, self.end.y]
		return rnd

	def get_best_last_index(self):
		disglist = [self.calc_dist_to_goal(node.x, node.y) for node in self.nodeList]
		goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
		#  print(goalinds)
		mincost = min([self.nodeList[i].cost for i in goalinds])
		for i in goalinds:
			if self.nodeList[i].cost == mincost:
				return i
		return None
	
	def gen_final_course(self, goalind):
		path = [[self.end.x, self.end.y]]
		# print "get final course param = ", self.nodeList[goalind].parent
		while self.nodeList[goalind].parent is not None:
			node = self.nodeList[goalind]
			path.append([node.x, node.y])
			goalind = node.parent
		path.append([self.start.x, self.start.y])
		return path
	
	def calc_dist_to_goal(self, x, y):
		return np.linalg.norm([x - self.end.x, y - self.end.y])
	
	def find_near_nodes(self, inpNode): 
		nnode = len(self.nodeList)
		gamma = 5 # planning constant
		r = gamma * math.sqrt((math.log(nnode) / nnode))
		plt.plot(inpNode.x + r*np.cos(t), inpNode.y + r*np.sin(t), "b", linewidth=.2)
		#  r = self.expandDis * 5.0
		dlist = [(node.x - inpNode.x) ** 2 + (node.y - inpNode.y) ** 2 for node in self.nodeList]
		nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
		# print "nearinds = ", nearinds, "\n\n"
		return nearinds
	
	def rewire(self, newNode, nearinds):
		nnode = len(self.nodeList)
		for i in nearinds:
			nearNode = self.nodeList[i]
			dx = newNode.x - nearNode.x
			dy = newNode.y - nearNode.y
			d = math.sqrt(dx ** 2 + dy ** 2)
			scost = newNode.cost + d
			tCost = newNode.time + abs(newNode.x - nearNode.x)/Vf
			
			if nearNode.cost > scost:
				theta = math.atan2(dy, dx)
				if self.check_collision_extend(nearNode, theta, d):
					nearNode.parent = nnode - 1
					nearNode.cost = scost
					nearNode.time = tCost
	
	def check_collision_extend(self, nearNode, theta, d):
		
		tmpNode = copy.deepcopy(nearNode)
		
		for i in range(int(d / self.expandDis)):
			tmpNode.x += self.expandDis * math.cos(theta)
			tmpNode.y += self.expandDis * math.sin(theta)
			if not self.__CollisionCheck(tmpNode, self.obstacleList):
				return False
		return True
	
	def drawObstacles(self, obstacleList):	
		# define obstacle
		# for (ox, oy, size) in self.obstacleList:
			# plt.plot(ox, oy, "ok", ms = 5 * size)

		# deltaX = 5
		# deltaY = 2
		# rearPoint = [10,2]
		
		for (rearPoint, inpDimension,inpVel) in obstacleList: 
			rec = [	[rearPoint[0],rearPoint[1]-inpDimension[1]/2],
					[rearPoint[0],rearPoint[1]+inpDimension[1]/2],
					[rearPoint[0]+inpDimension[0],rearPoint[1]+inpDimension[1]/2],
					[rearPoint[0]+inpDimension[0],rearPoint[1]-inpDimension[1]/2],
					[rearPoint[0],rearPoint[1]-inpDimension[1]/2]]

			recCenter = [rearPoint[0]+inpDimension[0]/2,rearPoint[1]]
			recP1 = [rearPoint[0],rearPoint[1]-inpDimension[1]/2]
			diagDist = math.sqrt((recCenter[0]-recP1[0])**2 + ((recCenter[1]-recP1[1])**2))

			# print "diagDist = ", diagDist

			plt.plot([x for (x, y) in rec], [y for (x, y) in rec], '-k',linewidth=.9)

			# plt.plot(rearPoint[0],rearPoint[1], "c^")
			# plt.plot(recCenter[0],recCenter[1], "r*")
			# plt.plot(recP1[0],recP1[1], "bo")


		# for (ox, oy, sizeA,inpVel) in self.obstacleList:
			# plt.plot(ox + sizeA*np.cos(t), oy + sizeA*np.sin(t), "k", linewidth=2.2)
	
	def DrawGraph(self, inputPoint=None):
		# plt.clf() # clear the figure
		if inputPoint is not None:
			plt.plot(inputPoint[0], inputPoint[1], "oy")
		for node in self.nodeList:
			if node.parent is not None:
				plt.plot([node.x, self.nodeList[node.parent].x],[node.y, self.nodeList[node.parent].y], "g--",linewidth=1)

		# plotting the initial and goal on the graph
		plt.plot(self.start.x, self.start.y, "ko")
		plt.plot(self.end.x, self.end.y, "bs")

	def getClosestNodeIdx(self, nodeList, rnd):
	
		# Distance wise:
		dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2 for node in nodeList]
		minDistanceNodeIdx = dlist.index(min(dlist))
		
		# Time wise:
		# tList = [abs(node.x - rnd[0]) / Vf for node in nodeList]
		tList = [math.sqrt((node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2) / Vf for node in nodeList]
		minTimeNodeIdx = tList.index(min(tList))
		
		# print "Distance = ", dlist, "idxD = ", minDistanceNodeIdx
		# print "\n\n"
		# print "Time = ", tList, "idxT = ", minTimeNodeIdx
		# print "----------------------------- "
		return [minDistanceNodeIdx,minTimeNodeIdx]
	
	def __CollisionCheck(self, node, obstacleList):
		
		if (len(obstacleList)==0):
			# print "SAFE"
			return True
		else:
			for (rear, dimension,inpVel) in obstacleList:
				recCenter = [rear[0]+dimension[0]/2,rear[1]]
				recP1 = [rear[0],rear[1]-dimension[1]/2]
				diagDist = math.sqrt((recCenter[0]-recP1[0])**2 + ((recCenter[1]-recP1[1])**2))

				predOX = recCenter[0] + node.time*inpVel # prediction of pose of obstacle
				dx = predOX - node.x
				dy = recCenter[1] - node.y
				nodeDiff = math.sqrt(dx * dx + dy * dy)
				# print "nodeDiff = ", nodeDiff
				if nodeDiff <= diagDist:
					# print "COLLISION"
					return False  # collision
				# print "predX = ", predOX, " , nodeTime = ", node.time, " , node X = ", node.x, " , node Y = ", node.y, "oy = ", rear[1]
			# return True  # safe

class Node():
    def __init__(self, x, y):
		self.x = x
		self.y = y
		self.cost = 0.0
		self.time = 0.0
		self.parent = None

def main():
	# ====Search Path with RRT====
	print(" -------------- RRT* planning -------------- ")
    
	x = np.linspace(0, LENGTH_FREEWAY,1000)
	Y_upp = [] 
	Y_dwn = []
	Y_lane1 = []
	Y_lane2 = []
	Y_lane3 = []
	for i in range (len(x)):
		Y_upp.append(WIDTH_FREEWAY)
		Y_dwn.append(0)
		
		Y_lane1.append(0+LANE_LENGH)
		Y_lane2.append(0+2*LANE_LENGH)
		
	plt.plot(x,Y_dwn,"-k")
	plt.plot(x,Y_upp,"-k")
	plt.plot(x,Y_lane1,"--k")
	plt.plot(x,Y_lane2,"--k")
	
	xSafe = 20 #m
	ySafe = 0 #m
	
	# Vehicle M:
	vehM = np.zeros((2,K+1),dtype=float)
	VmLong = V_m
	VmLat = 0
	vehMinit = [7,LANE_LENGH/2]
	vehM[0,0] = vehMinit[0]
	vehM[1,0] = vehMinit[1]
	
	# vehicle N:
	vehN = np.zeros((2,K+1),dtype=float)
	VnLong = VmLong
	VnLat = 0
	vehNinit = [vehMinit[0],vehMinit[1]+4]
	vehN[0,0] = vehNinit[0]
	vehN[1,0] = vehNinit[1]
	
	# Goal 
	goalCurr = np.zeros((2,K+1),dtype=float)
	# goalinit = [vehMinit[0]+xSafe+,vehMinit[1]+ySafe]
	goalinit = [xSafe + vehMinit[0],vehMinit[1] + ySafe]
	
	goalCurr[0,0] = goalinit[0]
	goalCurr[1,0] = goalinit[1]
	
	
	prevVehM = [vehM[0,0],vehM[1,0]]
	for i in range(K):
		vehM[0,i+1] = VmLong*Ts + prevVehM[0]
		vehM[1,i+1] = VmLat*Ts + prevVehM[1]
		
		goalCurr[0,i+1] = xSafe + vehM[0,i+1] 
		goalCurr[1,i+1] = ySafe + vehM[1,i+1]
		
		prevVehM[0] = vehM[0,i+1]
		prevVehM[1] = vehM[1,i+1]
	# --------------------------------------------
	
	prevVehN = [vehN[0,0],vehN[1,0]]
	for i in range(K):
		vehN[0,i+1] = VnLong*Ts + prevVehN[0]
		vehN[1,i+1] = VnLat*Ts + prevVehN[1]
		
		# goalCurr[0,i+1] = vehN[0,i+1] + xSafe
		# goalCurr[1,i+1] = vehN[1,i+1] + ySafe
		
		prevVehN[0] = vehN[0,i+1]
		prevVehN[1] = vehN[1,i+1]
		
	# --------------------------------------------
	# Set Initial parameters
	initPoint =[0,LANE_LENGH/2]
	start = initPoint
	tStart = time.time()
	for j in range(K+1):
		
		
		# Static Goal:
		goal =[50,2]
		
		# # Dynamic Goal:
		# goal = [Vf*Ts*j+goalinit[0],goalCurr[1,j]]
		randArea = [start[0], goal[0], 0, WIDTH_FREEWAY]
		
		# randArea = [start[0], goalCurr[0,j]]
		# goal = [goalCurr[0,j],goalCurr[1,j]]
		
		
		# # dynamic and static obstacle:
		# if j == int(K/6) or j == K-2:
			# print "------------------ Static Obstacle has been detected! ------------------"
			# obsTot = [(vehM[0,j],vehM[1,j],1,.2),(vehM[0,j]+1.4,vehM[1,j]+2.5,2,.5)]
		# else:
			# obsTot = [(vehM[0,j],vehM[1,j],1,.2)]
		
		# only dynamic obstacle:
		obsTot = []
		# obsTot = [([vehM[0,j],vehM[1,j]],[5,2],VmLong),([vehN[0,j],vehN[1,j]],[5,2],VnLong)]
		obsTot = [([vehM[0,j],vehM[1,j]],[5,2],VmLong)]
		
		print "len obst = ", len(obsTot)
		
		print "Vehicle Follower = ",start," , Vehicle M = ", obsTot, " , goal = ", goal , "\n\n"
		rrtS = RRTSTAR(start, goal,obsTot,randArea)
		# path = rrtS.Planning(animation=show_animation)
		path = rrtS.Planning(animation=False)
		
		print "path = ", path
		print "----------------------------------------------------------------"
		temp2 = path[-2]
		temp1 = path[-1]
		theta = math.atan(( temp2[1]- temp1[1])/( temp2[0] - temp1[0]))
		start = [Vf* math.cos(theta)*Ts+start[0],Vf*math.sin(theta)*Ts+start[1]]
		
		# Draw obstacles:
		rrtS.drawObstacles(obsTot)
		
		# Draw the generated path:
		rrtS.DrawGraph()
		
		# plt.plot(obsTot[0][0], obsTot[0][1], "ok", ms = 5 * obsTot[0][2])
		
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r',linewidth=.8)
		
		# plot the position of car in the next timeStep
		plt.plot(start[0],start[1],"^y")
	
	tEnd = time.time()
	tDiff = tEnd - tStart
	print "Running Time = ", tDiff, " [sec]"
	# plt.grid()
	plt.xlabel('X [m]')
	plt.ylabel('Y [m]')
	plt.autoscale(enable=True, axis=u'both', tight=False)
	plt.title('Trajectory')
	plt.axis([-5, goal[0]+100, 0-5, WIDTH_FREEWAY+5])
	plt.show()
	
	plt.savefig(results_dir + sample_file_name,dpi = 600)
	

if __name__ == '__main__':
    main()
