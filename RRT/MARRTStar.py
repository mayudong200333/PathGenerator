#MARRTStar
#made by mayudong

from RRT.GRRT import Graph,Node
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat

class MARRTStar():
    def __init__(self,startlis,goallis,randArea=[0,9],obstacleList=[(2,4),(2.5,4),(3,3),(3,3.5),(3,4.5),(3,5),(3,4),(3.5,4),(4,4),(4.5,4),(5,4),(5,3.5),(5,3),(5,2.5),(5,2)],goalsampleRate=20,maxIter=100,cmax=4):

        self.graph = Graph(size=(19, 19), xrange=randArea, yrange=randArea, obstacleList=obstacleList)
        self.graphnode = self.graph.nodeList

        self.agent_number = len(startlis)
        self.startlis = []
        self.goallis = []

        self.get_start_goal(startlis,goallis)

        self.goalsampleRate = goalsampleRate
        self.maxIter = maxIter
        self.cmax = cmax

        self.nodeList = [[start,] for start in self.startlis]


        px = [start.x for start in self.startlis]
        py = [start.y for start in self.startlis]

        self.graph.add_edge(px,py)



    def get_start_goal(self,startlis,goallis):
        for node in self.graphnode:
            try:
                for start in startlis:
                    if len(self.startlis) == self.agent_number:
                        break
                    if node.x == start[0] and node.y == start[1]:
                        self.startlis.append(copy.deepcopy(node))
                for goal in goallis:
                    if len(self.goallis) == self.agent_number:
                        break
                    if node.x == goal[0] and node.y == goal[1]:
                        self.goallis.append(copy.deepcopy(node))
            except:
                print('You should redefine the start or goal region')
        if len(self.startlis) != len(self.goallis):
            print('You should redefine the start or goal region')

    def planning(self):
        print('start planning')
        for i in self.maxIter:
            rndlis = self.sample()


    def sample(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = self.graphnode[random.randint(0,len(self.graphnode)-1)]
        else:
            rnd = self.goal
        return rnd















if __name__ == '__main__':
    start = [(4,3),(0,0)]
    goal = [(8,8),(7,8)]
    marrt = MARRTStar(start,goal)
    print(marrt.nodeList[0][0].y)
