"""
The path planning example for RRTSTar combined with dubins_path
"""

import copy
import math
import os
import random
import sys
import matplotlib.pyplot as plt
import numpy as np
from RRT.RRTStar import Node,RRTStar
from DubinsPath.dubins_path_planner import Dubin

class Node(Node):
    def __init__(self,x,y,yaw):
        super().__init__(x,y)
        self.yaw = yaw
        self.path_yaw = []

class DubinRRTStar(RRTStar):
    def __init__(self,start,goal,obstacleList,randArea,expanDis=0.5,goalSampleRate=20,maxIter=100,c=1.0):
        self.start = Node(start[0],start[1],start[2])
        self.end = Node(goal[0], goal[1],goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expanDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.nodeList = []
        self.obstacleList = obstacleList
        self.c = c
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5


    def Planning(self,animation = True):
        animation = False

        self.nodeList.append(self.start)
        for i in range(self.maxIter):
            rnd = self.sample()
            minind = self.NearestIndex(rnd)
            nearestNode = self.nodeList[minind]
            newNode = self.steer(nearestNode,rnd)
            if self.check_collision(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode,nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode,nearinds)

            if animation:
                self.DrawGraph(rnd)
        # generate path
        lastIndex = self.get_best_last_index()
        path = self.gen_final_path(lastIndex)
        return path

    def sample(self):
        if random.randint(0,100) > self.goalSampleRate:
            rnd = Node(random.uniform(self.minrand,self.maxrand),random.uniform(self.minrand,self.maxrand),random.uniform(-math.pi,math.pi))
        else:
            rnd = Node(self.end.x,self.end.y,self.end.yaw)
        return rnd

    def NearestIndex(self,rnd):
        # user can difine cost-function here
        # the cost function used here is 2-norm
        Jlist = [(node.x - rnd.x)**2 + (node.y - rnd.y)**2 for node in self.nodeList]
        minind = Jlist.index(min(Jlist))
        return minind

    def steer(self,start,end):
        startlis = [start.x,start.y,start.yaw]
        endlis = [end.x,end.y,end.yaw]
        DubinPath = Dubin(startlis,endlis,self.c)

        if len(DubinPath.px) <= 1: # can't find Dubins path
            return None

        newNode = copy.deepcopy(start)
        newNode.x = DubinPath.px[-1]
        newNode.y = DubinPath.py[-1]
        newNode.yaw = DubinPath.pyaw[-1]

        newNode.path_x = DubinPath.px
        newNode.path_y = DubinPath.py
        newNode.path_yaw = DubinPath.pyaw
        newNode.cost += DubinPath.length
        newNode.parent = start
        return newNode

    def get_best_last_index(self):
        # check dist
        disglist = [self.calc_dist_to_goal(node.x,node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.goal_xy_th]
        # check angle
        last_goalinds = [i for i in goalinds if abs(self.nodeList[i].yaw-self.end.yaw) <= self.goal_yaw_th]

        mincost = min([self.nodeList[i].cost for i in last_goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i
        return None

    def gen_final_path(self,goalind):
        path = [[self.end.x,self.end.y]]
        node = self.nodeList[goalind]
        while node.parent:
            for (ix,iy) in zip(reversed(node.path_x),reversed(node.path_y)):
                path.append([ix,iy])
            node = node.parent
        path.append([self.start.x,self.start.y])
        return path

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x,node.path_y, "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


    def check_collision(self,node):

        if node is None:
            return False

        for (ox, oy, size) in self.obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False  # collision

        return True  # safe


    def choose_parent(self,newNode,nearinds):
        if len(nearinds) == 0:
            return newNode

        Jlist = []
        for i in nearinds:
            nearNode = self.nodeList[i]
            t_node = self.steer(nearNode,newNode)
            if t_node and self.check_collision(t_node):
                Jlist.append(self.calc_new_cost(nearNode,newNode))
            else:
                Jlist.append(float("inf"))

        mincost = min(Jlist)
        minind = nearinds[Jlist.index(mincost)]


        if mincost == float("inf"):
            print("mincost is inf")
            return newNode


        newNode = self.steer(self.nodeList[minind],newNode)
        newNode.cost = mincost


        return newNode

    def rewire(self,newNode,nearinds):
        for i in nearinds:
            nearNode = self.nodeList[i]
            edgeNode = self.steer(newNode,nearNode)
            if not edgeNode:
                continue
            edgeNode.cost = self.calc_new_cost(newNode,nearNode)

            if nearNode.cost > edgeNode.cost:
                if self.check_collision(edgeNode):
                    nearNode.parent =edgeNode.parent
                    nearNode.cost = edgeNode.cost
                    nearNode.x = edgeNode.x
                    nearNode.y = edgeNode.y
                    nearNode.path_x = edgeNode.path_x
                    nearNode.path_y = edgeNode.path_y

    def calc_new_cost(self, start, end):
        start1 = [start.x,start.y,start.yaw]
        end1 = [end.x,end.y,end.yaw]
        DubinPath = Dubin(start1, end1, self.c)
        return start.cost + DubinPath.length



if __name__ == '__main__':
    print("Start rrt start planning")

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = DubinRRTStar(start=[0, 0,np.deg2rad(0.0)], goal=[10, 10,np.deg2rad(45.0)],
                  randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=True)
    print(path)
    #nodelist = rrt.nodeList
    #for i in nodelist:
    #    print(i.parent)


    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()






