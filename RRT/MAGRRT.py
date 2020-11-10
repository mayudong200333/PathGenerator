#MARRTStar
#a new algorithm made by mayudong

from RRT.GRRT import Graph,Node,GRRT
import copy
import random
import math
import numpy as np
import functools
import operator
import matplotlib.pyplot as plt

class MAGRRT:
    def __init__(self,startlis,goallis,randArea=(0,9),size=(19,19),obstacleList=[(2,4),(2.5,4),(3,3),(3,3.5),(3,4.5),(3,5),(3,4),(3.5,4),(4,4),(4.5,4),(5,4),(5,3.5),(5,3),(5,2.5),(5,2)],goalSampleRate=20,maxIter=100,cmax=6,animation=True):

        self.n_agent = len(startlis)
        self.startlis = startlis
        self.goallis = goallis
        self.randArea =randArea
        self.obstacleList = obstacleList
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.cmax = cmax
        self.size = size
        self.animation = animation
        self.agentPathList = []
        self.agentPathList_last = []

    def planning(self):
        ob_first = copy.copy(self.obstacleList)

        j = 0
        for ob_add in self.goallis:
            if j == 0:
                j += 1
                continue
            ob_first.append(ob_add)
            j += 1
        print(ob_first)
        first_agent = GRRT(self.startlis[0],self.goallis[0],obstacleList=ob_first,size=self.size,xrange=self.randArea,yrange=self.randArea,goalSampleRate=self.goalSampleRate,maxIter=self.maxIter,cmax=self.cmax,animation=self.animation)
        first_agent.planning()
        first_agent_path = [first_agent.px,first_agent.py]
        self.agentPathList.append(first_agent_path)
        for i in range(1,self.n_agent):
            obi = copy.copy(self.obstacleList)
            j = 0
            for ob_add in self.goallis:
                if j == i:
                    j += 1
                    continue
                if ob_add != self.startlis[i]:
                    obi.append(ob_add)
                else:
                    print('You should check goal area')
                j += 1
            print(self.startlis[i],self.goallis[i])
            agenti = GRRT(self.startlis[i],self.goallis[i],obstacleList=obi,size=self.size,xrange=self.randArea,yrange=self.randArea,agentpath=self.agentPathList,goalSampleRate=self.goalSampleRate,maxIter=self.maxIter,cmax=self.cmax,animation=self.animation)
            agenti.int_planning()
            agenti_path = [agenti.px,agenti.py]
            self.agentPathList.append(agenti_path)


if __name__ == '__main__':
    start = [(4, 1), (4, 8)]
    goal = [(4, 8), (4, 5)]
    #start = [(8, 1), (1, 7), (1, 1), (4, 1), (8.5, 1)]
    #goal = [(1,8),(6,1),(7,8),(4,8),(5.5,8)]
    #start = [(4, 1), (8.5, 1), (1, 7), (1, 1), (8, 1),(0,0)]
    #goal = [(4,8),(5.5,8),(6,1),(7,8),(1,8),(9,9)]
    obstacleList = [(2, 4), (2.5, 4), (2, 4.5), (2.5, 4.5), (2.5, 5), (2, 5), (4.5, 5), (4.5, 5.5), (5, 5), (5.5, 5),
                    (5, 5.5), (5.5, 5.5), (2, 6), (2.5, 6), (2, 6.5), (2.5, 6.5), (7, 4), (7.5, 4), (7, 4.5),
                    (7.5, 4.5), (6, 7), (6.5, 7.5), (6, 7.5), (6.5, 7)]
    graph = Graph(obstacleList=obstacleList)
    graph.drawgraph()
    marrt = MAGRRT(start, goal, obstacleList=obstacleList)
    marrt.planning()
    pathlist = marrt.agentPathList


    a0 = pathlist[0][0]
    b0 = pathlist[0][1]
    a1 = pathlist[1][0]
    b1 = pathlist[1][1]
    #a2 = pathlist[2][0]
    #b2 = pathlist[2][1]
    #a3 = pathlist[3][0]
    #b3 = pathlist[3][1]
    #a4 = pathlist[4][0]
    #b4 = pathlist[4][1]
    #a5 = pathlist[5][0]
    #b5 = pathlist[5][1]




    
    for i in range(max(len(a0),len(a1))):
        try:
            plt.plot([a0[i],a0[i+1]],[b0[i],b0[i+1]],color='red')
        except:
            pass
        try:
            plt.plot([a1[i], a1[i + 1]], [b1[i], b1[i + 1]],color='gray')
        except:
            pass
        '''
        try:
            plt.plot([a2[i], a2[i + 1]], [b2[i], b2[i + 1]],color='black')
        except:
            pass
        try:
            plt.plot([a3[i], a3[i + 1]], [b3[i], b3[i + 1]],color='blue')
        except:
            pass
        try:
            plt.plot([a4[i], a4[i + 1]], [b4[i], b4[i + 1]],color='purple')
        except:
            pass

        try:
            plt.plot([a5[i], a5[i + 1]], [b5[i], b5[i + 1]],color='purple')
        except:
            pass
        '''
        plt.pause(0.7)
    plt.show()

    # obstacle list に　cos　大きいものを追加しよう



