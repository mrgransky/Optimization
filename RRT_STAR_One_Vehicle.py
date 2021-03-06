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

results_dir = os.path.join(os.getcwd(), 'Results/Jan25/')
sample_file_name = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + "path.jpeg"

t = np.linspace(0, 2*np.pi,1000)
K = 5
Ts = 1

# vehicle Follower(F)
Vf = 8

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

class RRTSTAR():
	def __init__(self, start, goal, obstacleList, randArea,expandDis=0.5, goalSampleRate=50, maxIter=300):
		self.start = Node(start[0], start[1])
		self.end = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
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
					selNode = self.steer(rnd, minDisIdx, minTimeIdx)
					if self.__CollisionCheck(selNode, self.obstacleList):
						nearinds = self.find_near_nodes(selNode)
						
						newNode = self.choose_parent(selNode, nearinds)
						
						self.nodeList.append(newNode)
						
						self.rewire(newNode, nearinds)
					newNode2Point = [newNode.x,newNode.y]
					if animation:
						self.DrawGraph(rnd)
			lastIndex = self.get_best_last_index()
			path = self.gen_final_course(lastIndex)
			return path

	def choose_parent(self, inpNode, nearinds):
		if len(nearinds) == 0:
			return inpNode
		costList = []
		for i in nearinds:
			dx = inpNode.x - self.nodeList[i].x
			dy = inpNode.y - self.nodeList[i].y
			d = math.sqrt(dx ** 2 + dy ** 2)
			theta = math.atan2(dy, dx)
			if self.check_collision_extend(self.nodeList[i], theta, d):
				# print "cost = ", self.nodeList[i].cost, " , distance = " , d
				costList.append(self.nodeList[i].cost + d)
			else:
				costList.append(float("inf"))
		# print "Cost = ", costList
		mincost = min(costList)
		minCostNodeIdx = nearinds[costList.index(mincost)]

		if mincost == float("inf"):
			print("mincost is inf")
			return inpNode

		inpNode.cost = mincost
		inpNode.parent = minCostNodeIdx

		# print "newNode_2 = (", inpNode.x, ",", inpNode.y,") , Cost = ", inpNode.cost, " , parent = ", inpNode.parent, "\n\n"

		return inpNode

	def steer(self, rnd, inpMinDisNodeIdx,inpMinTimeNodeIdx):
		nearestNodeDis = self.nodeList[inpMinDisNodeIdx]
		nearestNodeTime = self.nodeList[inpMinTimeNodeIdx]
		# slope
		theta = math.atan2(	rnd[1] - nearestNodeTime.y, rnd[0] - nearestNodeTime.x)
		
		newNode = copy.deepcopy(nearestNodeTime)
		
		newNode.x += self.expandDis * math.cos(theta)
		newNode.y += self.expandDis * math.sin(theta)
		
		newNode.cost += self.expandDis
		newNode.parent = inpMinTimeNodeIdx
		# print "newNode = (", newNode.x, ",", newNode.y,") , Cost = ", newNode.cost, " , parent = ", newNode.parent, "\n"
		
		return newNode
	
	def get_random_point(self):
		if random.randint(0, 100) > self.goalSampleRate:
			rnd = [random.uniform(self.minrand, self.maxrand),random.uniform(self.end.y - 1 ,self.end.y + 4)]
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

	def find_near_nodes(self, inpNode): # find the nearest vertices
		nnode = len(self.nodeList)
		r = 50.0 * math.sqrt((math.log(nnode) / nnode))
		#  r = self.expandDis * 5.0
		dlist = [(node.x - inpNode.x) ** 2 +(node.y - inpNode.y) ** 2 for node in self.nodeList]
		nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
		return nearinds

	def rewire(self, newNode, nearinds):
		nnode = len(self.nodeList)
		for i in nearinds:
			nearNode = self.nodeList[i]
			dx = newNode.x - nearNode.x
			dy = newNode.y - nearNode.y
			d = math.sqrt(dx ** 2 + dy ** 2)
			scost = newNode.cost + d

			if nearNode.cost > scost:
				theta = math.atan2(dy, dx)
				if self.check_collision_extend(nearNode, theta, d):
					nearNode.parent = nnode - 1
					nearNode.cost = scost

	def check_collision_extend(self, nearNode, theta, d):

		tmpNode = copy.deepcopy(nearNode)

		for i in range(int(d / self.expandDis)):
			tmpNode.x += self.expandDis * math.cos(theta)
			tmpNode.y += self.expandDis * math.sin(theta)
			if not self.__CollisionCheck(tmpNode, self.obstacleList):
				return False
		return True


	def drawObstacles(self):

		# define obstacle
		# for (ox, oy, size) in self.obstacleList:
			# plt.plot(ox, oy, "ok", ms = 5 * size)
		for (ox, oy, sizeA, sizeB) in self.obstacleList:
			plt.plot(ox + sizeA*np.cos(t), oy + sizeB*np.sin(t), "k", linewidth=2.2)

	def DrawGraph(self, inputPoint=None):
		# plt.clf() # clear the figure
		if inputPoint is not None:
			plt.plot(inputPoint[0], inputPoint[1], "oy")
		for node in self.nodeList:
			if node.parent is not None:
				plt.plot([node.x, self.nodeList[node.parent].x],[node.y, self.nodeList[node.parent].y], "g--",linewidth=.3)

		# plotting the initial and goal on the graph
		plt.plot(self.start.x, self.start.y, "co")
		plt.plot(self.end.x, self.end.y, "bo")

	def getClosestNodeIdx(self, nodeList, rnd):

		# Distance wise:
		dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2 for node in nodeList]
		minDistanceNodeIdx = dlist.index(min(dlist))

		# Time wise:
		tList = [abs(node.x - rnd[0]) / Vf for node in nodeList]
		minTimeNodeIdx = tList.index(min(tList))

		# print "Distance = ", dlist, "idxD = ", minDistanceNodeIdx
		# print "\n\n"
		# print "Time = ", tList, "idxT = ", minTimeNodeIdx
		# print "----------------------------- "
		return [minDistanceNodeIdx,minTimeNodeIdx]

	def __CollisionCheck(self, node, obstacleList):

		for (ox, oy, sizeA, sizeB) in obstacleList:
			dx = ox - node.x
			dy = oy - node.y
			d = dx * dx + dy * dy
			if d <= sizeA ** 2 or d <= sizeB:
				return False  # collision
		return True  # safe
class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def main():
	# ====Search Path with RRT====
		print(" -------------- RRT* planning -------------- ")


		# Set Initial parameters
		initPoint =[0,1]
		start = initPoint

		xSafe = 10 #m
		ySafe = 0 #m

		# Vehicle M:
		vehM = np.zeros((2,K+1),dtype=float)
		VmLong = 10
		VmLat = 0
		vehMinit = [7,1]
		vehM[0,0] = vehMinit[0]
		vehM[1,0] = vehMinit[1]

		# Dynamic Goal
		# goalCurr = np.zeros((2,K+1),dtype=float)
		# goalinit = [vehNinit[0]+xSafe,vehNinit[1]+ySafe]
		# goalCurr[0,0] = goalinit[0]
		# goalCurr[1,0] = goalinit[1]


		# Static Goal
		goalCurr = [40,1]

		prevVehM = [vehM[0,0],vehM[1,0]]
		for i in range(K):
			vehM[0,i+1] = VmLong*Ts + prevVehM[0]
			vehM[1,i+1] = VmLat*Ts + prevVehM[1]

			# goalCurr[0,i+1] = xSafe # + vehM[0,i+1]
			# goalCurr[1,i+1] = ySafe # + #vehM[1,i+1] + ySafe

			prevVehM[0] = vehM[0,i+1]
			prevVehM[1] = vehM[1,i+1]

		print "vehicle M = ", vehM
		print "Current Goal = ", goalCurr

		goal = goalCurr
		tStart = time.time()
		for j in range(K+1):
			# randArea = [start[0], goalCurr[0,j]]
			# goal = [goalCurr[0,j],goalCurr[1,j]]

			randArea = [start[0], goalCurr[0]]
			# # dynamic and static obstacle:
			# if j == int(K/6) or j == K-2:
				# print "------------------ Static Obstacle has been detected! ------------------"
				# obsTot = [(vehM[0,j],vehM[1,j],1,.2),(vehM[0,j]+1.4,vehM[1,j]+2.5,2,.5)]
			# else:
				# obsTot = [(vehM[0,j],vehM[1,j],1,.2)]

			# only dynamic obstacle:
			obsTot = [(vehM[0,j],vehM[1,j],1,.2)]#,(vehN[0,j],vehN[1,j],1.4,.3)]
			print "Current Obstacle = ", obsTot, " , goal = ", goal
			rrtS = RRTSTAR(start, goal,obsTot,randArea)
			# path = rrtS.Planning(animation=show_animation)
			path = rrtS.Planning(animation=False)

			# Draw obstacles:
			rrtS.drawObstacles()

			# Draw the generated path:
			rrtS.DrawGraph()
			print "path = ", path
			temp2 = path[-2]
			temp1 = path[-1]
			theta = math.atan(( temp2[1]- temp1[1])/( temp2[0]- temp1[0]))
			start = [Vf* math.cos(theta)*Ts+start[0],Vf*math.sin(theta)*Ts+start[1]]
			plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r',linewidth=1.2)
			
			# plot the position of car in the next timeStep
			plt.plot(start[0],start[1],"^y")

		tEnd = time.time()
		tDiff = tEnd - tStart
		print "Running Time = ", tDiff, " [sec]"


		plt.grid()
		plt.xlabel('X [m]')
		plt.ylabel('Y [m]')
		plt.title('Trajectory')
		plt.axis([-7, goal[0]+50, -1, goal[1]+35])
		plt.show()

		plt.savefig(results_dir + sample_file_name,dpi = 600)
if __name__ == '__main__':
    main()
