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

if not os.path.isdir(results_dir):
    os.makedirs(results_dir)

class RRTSTAR():
	def __init__(self, start, goal, obstacleList, randArea,expandDis=0.5, goalSampleRate=20, maxIter=2000):
		self.start = Node(start[0], start[1])
		self.end = Node(goal[0], goal[1])
		self.minrand = randArea[0]
		self.maxrand = randArea[1]
		self.expandDis = expandDis
		self.goalSampleRate = goalSampleRate
		self.maxIter = maxIter
		self.obstacleList = obstacleList

	def Planning(self, animation=True):
	
	
	# for i in range(K+1):
	
			self.nodeList = [self.start]
			for i in range(self.maxIter):
				# print "nodeList = "
				# for node in self.nodeList:
					# print "(x,y) = (", node.x, ",", node.y, ")"
			
					# print "------------------------------------------"
					rnd = self.get_random_point()
					# print "random point =",  rnd
			
			
					nearestNode2RandNodeIdx = self.GetNearestListIndex(self.nodeList, rnd)
					# print "idx nearest = ", nearestNode2RandNodeIdx
			
			
					newNode = self.steer(rnd, nearestNode2RandNodeIdx)
					#  print(newNode.cost)
					# print "new Node = (", newNode.x, ",", newNode.y,")"
					# print "------------------------------------------"
			
			
					if self.__CollisionCheck(newNode, self.obstacleList):
						nearinds = self.find_near_nodes(newNode)
						newNode = self.choose_parent(newNode, nearinds)
						self.nodeList.append(newNode)
						self.rewire(newNode, nearinds)
				
					newNode2Point = [newNode.x,newNode.y]
					if animation:
						self.DrawGraph(rnd)
					# self.DrawGraph(newNode2Point)
			# generate coruse
		
			lastIndex = self.get_best_last_index()
			path = self.gen_final_course(lastIndex)
			# print "path = ", path
			return path
			

	def choose_parent(self, newNode, nearinds):
		
		if len(nearinds) == 0:
			return newNode
		dlist = []
		for i in nearinds:
			dx = newNode.x - self.nodeList[i].x
			dy = newNode.y - self.nodeList[i].y
			d = math.sqrt(dx ** 2 + dy ** 2)
			theta = math.atan2(dy, dx)
			if self.check_collision_extend(self.nodeList[i], theta, d):
				dlist.append(self.nodeList[i].cost + d)
			else:
				dlist.append(float("inf"))
		mincost = min(dlist)
		minearestNode2RandNodeIdx = nearinds[dlist.index(mincost)]
		
		if mincost == float("inf"):
			print("mincost is inf")
			return newNode
		
		newNode.cost = mincost
		newNode.parent = minearestNode2RandNodeIdx
		return newNode
	
	def steer(self, rnd, nearestNode2RandNodeIdx):
		nearestNode2RandNode = self.nodeList[nearestNode2RandNodeIdx]
		# print "nearest Node = (", nearestNode2RandNode.x, " , ", nearestNode2RandNode.y, ") , idx = ", nearestNode2RandNodeIdx
		# print "---time to move from random node to nearest node---"
		
		# slope
		theta = math.atan2(	rnd[1] - nearestNode2RandNode.y, 
							rnd[0] - nearestNode2RandNode.x)
		
		newNode = copy.deepcopy(nearestNode2RandNode)
		
		newNode.x += self.expandDis * math.cos(theta)
		newNode.y += self.expandDis * math.sin(theta)
		
		newNode.cost += self.expandDis
		
		newNode.parent = nearestNode2RandNodeIdx
		
		return newNode
	
	def get_random_point(self):
		if random.randint(0, 100) > self.goalSampleRate:
			rnd = [random.uniform(self.minrand, self.maxrand),random.uniform(self.minrand, 2)]
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
	
	def find_near_nodes(self, newNode):
		nnode = len(self.nodeList)
		r = 50.0 * math.sqrt((math.log(nnode) / nnode))
		#  r = self.expandDis * 5.0
		dlist = [(node.x - newNode.x) ** 2 +(node.y - newNode.y) ** 2 for node in self.nodeList]
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
	
	def DrawGraph(self, inputPoint=None):
		plt.clf() # clear the figure
		if inputPoint is not None:
			plt.plot(inputPoint[0], inputPoint[1], "oy")
		for node in self.nodeList:
			if node.parent is not None:
				# print " printed node = (", node.x, ",", node.y, ")"," , node parent = ", node.parent
				# draw the line between 
				plt.plot(	[node.x, self.nodeList[node.parent].x], 
							[node.y, self.nodeList[node.parent].y], "-b")
		
		
		# define obstacle
		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms = 5 * size)
			
		# plotting the initial and goal on the graph
		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.end.x, self.end.y, "xr")
		
		# plt.axis([-1, 16, -1, 5])
		# plt.grid(True)
		# plt.pause(0.01)

	def GetNearestListIndex(self, nodeList, rnd):
		dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])** 2 for node in nodeList]
		minearestNode2RandNodeIdx = dlist.index(min(dlist))
		# print "Distance = ", dlist, "idx = ", minearestNode2RandNodeIdx
		return minearestNode2RandNodeIdx
	
	def __CollisionCheck(self, node, obstacleList):
		for (ox, oy, size) in obstacleList:
			dx = ox - node.x
			dy = oy - node.y
			d = dx * dx + dy * dy
			if d <= size ** 2:
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
    
	
	
	K = 2
	Ts = 1
	
	
	# Vehicle M:
	sizeM = 1
	VmLong = 4
	VmLat = 0
	vehMinit = [7,1]
	vehM = np.zeros((2,K+1),dtype=float)

	vehM[0,0] = vehMinit[0]
	vehM[1,0] = vehMinit[1]
	
	prevVehM = [vehM[0,0],vehM[1,0]]
	for i in range(K):
		vehM[0,i+1] = VmLong*Ts + prevVehM[0]
		vehM[1,i+1] = VmLat*Ts + prevVehM[1]
		prevVehM[0] = vehM[0,i+1]
		prevVehM[1] = vehM[1,i+1]
	# print "vehM = ", vehM
	
	
	# vehicle Follower(F)
	Vf = 1.4
	
	
	# obsM = []
	# # obsM.append((30,0,2))
	
	# for j in range(K+1):
		# obsM.append((vehM[0,j],vehM[1,j],1))

	
    # Set Initial parameters
	
	initPoint =[1,1]
	# start = [1, 1]
	goal =  [vehM[0,-1]+8, vehM[1,-1]+3]# needs modifications
	
	# make an obj to the RRT* class
	# rrtS = RRTSTAR(start, goal,obstacleList,randArea)
	
	start = initPoint
	for j in range(K+1):
		randArea = [start[0]-2, goal[0]+2]
		obsM=[(vehM[0,j],vehM[1,j],1)]
		print "obsM = ", obsM
		rrtS = RRTSTAR(start, goal,obsM,randArea)
		# path = rrt.Planning(animation=show_animation)
		path = rrtS.Planning(animation=False) 
		
		print "path = ", path
		temp2 = path[-2]
		temp1 = path[-1]
		theta = math.atan(( temp2[1]- temp1[1])/( temp2[0]- temp1[0]))
		start = [Vf* math.cos(theta)*Ts+start[0],Vf*math.sin(theta)*Ts+start[1]]
		
		rrtS.DrawGraph()
		plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
		plt.plot(start[0],start[1],"^y")
		
		# plt.hold(True)
	
	plt.show()
	# tStart = time.time()
	# tEnd = time.time()
	# tDiff = tEnd - tStart
	# print "time = ", tDiff
	
	
	plt.savefig(results_dir + sample_file_name,dpi = 600)
	
	# Draw final path
	# if show_animation:
		
	# plt.grid(True)
	# plt.pause(0.01)  # Need for Mac
		
	

if __name__ == '__main__':
    main()
