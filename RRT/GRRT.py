# the base of the MARRT
# author mayudong

import random
import math
import copy
import _pickle as cPickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import sys

sys.setrecursionlimit(10000)

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.cost = 0
        self.px = []
        self.py = []
        self.parent = None
        self.children = []

class Graph:
    def __init__(self,size=(19,19),xrange=(0,9),yrange=(0,9),figure=False,obstacleList=[(2,2),(3,3)]):
        self.size = size
        self.graphMatrix = np.zeros(size) #initial the graph 0:node(connected) 1:path 2:obstacle
        self.xrange = xrange
        self.yrange = yrange

        self.obstacleList = obstacleList
        self.nodeList = []

        self.px = np.linspace(xrange[0],xrange[1],self.size[1])
        self.py = np.linspace(yrange[0],yrange[1],self.size[0])

        self.dx = self.px[1]-self.px[0]
        self.dy = self.py[1]-self.py[0]

        self.xx,self.yy = np.meshgrid(self.px,self.py)
        self.set_obstacle()
        self.set_node()
        self.connect_node()

        self.figure = figure
        if self.figure:
            self.drawgraph()

    def set_node(self):
        xx = np.flipud(self.xx)
        yy = np.flipud(self.yy)
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.graphMatrix[i][j] == 0:
                    x = xx[i][j]
                    y = yy[i][j]
                    node = Node(x,y)
                    self.nodeList.append(node)
                elif self.graphMatrix[i][j] == 2:
                    self.nodeList.append(2)

    def connect_node(self):
        indr = np.linspace(self.size[0],self.size[0]*(self.size[1]-1),self.size[1]-1,dtype=int)-1
        indl = self.size[0] * self.size[1] - 1
        indu = np.linspace(indr[-1]+1,indl-1,self.size[0]-1,dtype=int)
        i=0
        poplist = []
        for node in self.nodeList:
            if node == 2:
                poplist.append(i)
                #print(i)
                #print(self.nodeList[i])
                i += 1
                continue
            if i in indr:
                if self.nodeList[i+self.size[0]] != 2:
                    node.children.append(self.nodeList[i+self.size[0]])
                    self.nodeList[i+self.size[0]].children.append(node)
                i += 1
            elif i == indl:
                #print(i)
                continue
            elif i in indu:
                if self.nodeList[i + 1] != 2:
                    node.children.append(self.nodeList[i+1])
                    self.nodeList[i+1].children.append(node)
                i += 1
            else:
                if self.nodeList[i + self.size[0]] != 2:
                    node.children.append(self.nodeList[i + self.size[0]])
                    self.nodeList[i + self.size[0]].children.append(node)
                if self.nodeList[i + 1] != 2:
                    node.children.append(self.nodeList[i + 1])
                    self.nodeList[i + 1].children.append(node)
                i += 1
        for i in sorted(poplist, reverse=True):
            self.nodeList.pop(i)



    def calc_node_dis(self,node1,node2):
        dx = node1.x-node2.x
        dy = node1.y-node2.y
        return np.sqrt(dx**2+dy**2)


    def set_obstacle(self):
        for x,y in self.obstacleList:
            ix = int(x/self.dx)
            iy = int(self.size[0] - (y/self.dy + 1))
            ly = int(y/self.dy)
            self.graphMatrix[iy][ix] = 2
            self.xx[ly][ix] = float('inf')
            self.yy[ly][ix] = float('inf')


    def add_edge(self,px,py):
        for i,j in zip(px,py):
            ix = int(i/self.dx)
            iy = int(self.size[0] - (j/self.dy + 1))
            if self.check_add_edge(i,j):
                print('not allowed to add edge')
                continue
            self.graphMatrix[iy][ix] = 1

    def del_edge(self,px,py):
        for i,j in zip(px,py):
            ix = int(i/self.dx)
            iy = int(self.size[0] - (j/self.dy + 1))
            self.graphMatrix[iy][ix] = 0

    def check_add_edge(self,x,y):
        ix = int(x/self.dx)
        iy = int(self.size[0]-(y/self.dy+1))
        if self.graphMatrix[iy][ix] == 2:
            return True
        return False


    def drawgraph(self):
        plt.figure()
        plt.scatter(self.xx,self.yy)
        ax = plt.subplot()
        self.draw_obstacle(ax)
        self.search_pair()
        plt.xlim(self.xrange[0]-1,self.xrange[1]+1)
        plt.ylim(self.yrange[0]-1,self.yrange[1]+1)
        plt.show()


    def draw_obstacle(self,ax):
        for x,y in self.obstacleList:
            xs = x - self.dx/2
            ys = y - self.dy/2
            rec = pat.Rectangle(xy=(xs, ys), width=self.dx, height=self.dy,
                                angle=0, color="black")
            ax.add_patch(rec)

    def search_pair(self):
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.graphMatrix[i][j] == 1:
                    if i == self.size[0]-1 and j == self.size[1]-1:
                        continue
                    if i == self.size[0]-1:
                        if self.graphMatrix[i][j+1] == 1:
                            x1 = self.yrange[0] + j * self.dx
                            x2 = x1 + self.dx
                            y1 = self.xrange[0] + (self.size[0]-i-1) * self.dy
                            y2 = y1
                            plt.plot((x1,x2),(y1,y2),color='#000000')
                    elif j == self.size[1]-1:
                        if self.graphMatrix[i+1][j] == 1:
                            x1 = self.yrange[0] + j * self.dx
                            x2 = x1
                            y1 = self.xrange[0] + (self.size[0] - i - 1) * self.dy
                            y2 = y1 - self.dy
                            plt.plot((x1,x2),(y1,y2),color='#000000')
                    else:
                        if self.graphMatrix[i][j+1] == 1:
                            x1 = self.yrange[0] + j * self.dx
                            x2 = x1 + self.dx
                            y1 = self.xrange[0] + (self.size[0]-i-1) * self.dy
                            y2 = y1
                            plt.plot((x1,x2),(y1,y2),color='#000000')
                        if self.graphMatrix[i+1][j] == 1:
                            x1 = self.yrange[0] + j * self.dx
                            x2 = x1
                            y1 = self.xrange[0] + (self.size[0] - i - 1) * self.dy
                            y2 = y1 - self.dy
                            plt.plot((x1,x2),(y1,y2),color='#000000')


class GRRT:
    def __init__(self,start,goal,randArea=[0,10],obstacleList=[(2,4),(2.5,4),(3,3),(3,3.5),(3,4.5),(3,5),(3,4),(3.5,4),(4,4),(4.5,4),(5,4),(5,3.5),(5,3),(5,2.5),(5,2)],goalSampleRate=20,maxIter=100,cmax=4):

        self.graph = Graph(size=(19, 19), xrange=(0, 9), yrange=(0, 9), obstacleList=obstacleList)
        self.graphnode = self.graph.nodeList

        self.start = None
        self.goal = None

        self.get_start_goal(start,goal)

        self.obstacleList = obstacleList

        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.minrand = randArea[0]
        self.maxrand = randArea[1]

        self.nodeList = [self.start,]
        self.cmax = cmax

        px = [self.start.x]
        py = [self.start.y]

        self.graph.add_edge(px,py)

    def get_start_goal(self,start,goal):
        for node in self.graphnode:
            if self.start != None and self.goal != None:
                break
            if node.x == start[0] and node.y == start[1]:
                self.start = node
            elif node.x == goal[0] and node.y == goal[1]:
                self.goal = node
        if self.start == None or self.goal == None:
            print('You should redefine the start or goal region')

    def planning(self,animation=False):
        print('start planning')
        for i in range(self.maxIter):
            rnd = self.sample()

            xnearest = self.nearest(rnd)

            xnew,pnew = self.greedy(xnearest,rnd)

            #print(rnd.x,rnd.y)
            #print(xnew.x,xnew.y)
            #print(xnew.px)
            #print(xnew.py)

            if len(pnew)!=0:
                self.nodeList.append(xnew)
                xmin = xnearest
                Xnear = self.near(xnew)
                #print(Xnear)
                xminend = None
                for xnear in Xnear:
                    x_,p_ = self.greedy(xnear,xnew)
                    if self.calc_cost(x_,xnew)==0 :
                        c_ = x_.cost
                        #print('in')
                        #print(c_,xnew.cost)
                        if c_ < xnew.cost:
                            #print('in')
                            xmin = xnear
                            xminend = x_

                self.nodeList[self.nodeList.index(xnew)].parent = xmin


                if xminend:
                    self.nodeList[self.nodeList.index(xnew)].px = xminend.px
                    self.nodeList[self.nodeList.index(xnew)].py = xminend.py
                    self.nodeList[self.nodeList.index(xnew)].cost = xminend.cost

                try:
                    Xnear.pop(Xnear.index(xmin))
                except:
                    continue
                for xnear in Xnear:
                    x__,p__=self.greedy(xnew,xnear)
                    if self.calc_cost(x__,xnear)==0 and xnear.cost > x__.cost:
                        #print('in')
                        #print(self.nodeList.index(xnear))
                        #print(self.nodeList[self.nodeList.index(xnear)].px==x__.px)
                        self.nodeList[self.nodeList.index(xnear)].parent = xnew
                        self.nodeList[self.nodeList.index(xnear)].px = x__.px
                        self.nodeList[self.nodeList.index(xnear)].py = x__.py
                        self.nodeList[self.nodeList.index(xnear)].cost = x__.cost


        self.generate_path()


    def sample(self):
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = self.graphnode[random.randint(0,len(self.graphnode)-1)]
        else:
            rnd = self.goal
        return rnd

    def nearest(self,rnd):
        Jlist = [(node.x-rnd.x)**2+(node.y-rnd.y)**2 for node in self.nodeList]
        neari = Jlist.index(min(Jlist))
        xnearest = self.nodeList[neari]
        return xnearest

    def greedy(self,s,d):
        x = s
        c = 0
        path = []
        px = []
        py = []
        if (x.x-d.x)**2 + (x.y-d.y)**2 == 0:
            return x,path
        while c<=self.cmax:
            if (x.x-d.x)**2 + (x.y-d.y)**2 == 0:
                break
            childrenDisLis = [self.hueristic(child,d) for child in x.children]
            x_ = x.children[childrenDisLis.index(min(childrenDisLis))]
            c += self.calc_cost(x,x_)
            path.append(x_)
            px.append(x_.x)
            py.append(x_.y)
            x = x_
        sx = Node(x.x,x.y)
        sx.px = px
        sx.py = py
        sx.cost = s.cost + c
        sx.children = x.children
        return sx,path

    def hueristic(self,node1,node2):
        # you can difine your own hueristic here
        # but now we use L2-norm defined in graph caluc_dis
        dis = self.graph.calc_node_dis(node1,node2)
        return dis

    def calc_cost(self,node1,node2):
        # you can define your own cost function here
        # but now we use L2-norm defined in graph caluc_dis
        cost = self.graph.calc_node_dis(node1,node2)
        return cost

    def near(self,newNode):
        nearNodes = []
        nnode = len(self.nodeList)
        r = max(50.0 * math.sqrt((math.log(nnode) / nnode)),4)
        Jlist = [(node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [Jlist.index(i) for i in Jlist if i <= r ** 2 and i != 0]
        for i in nearinds:
            nearNodes.append(self.nodeList[i])
        return nearNodes

    def generate_path(self):
        i = 0
        px = []
        py = []
        for node in self.nodeList:
            if node.x == self.goal.x and node.y == self.goal.y:
                goalind = i
                break
            i += 1
        bestgoalnode = self.nodeList[goalind]
        for pxs,pys in zip(bestgoalnode.px,bestgoalnode.py):
            px.append(pxs)
            py.append(pys)
        while bestgoalnode.parent is not None:
            print('genrating now')
            preparent = bestgoalnode.parent.parent
            if preparent == bestgoalnode.parent:
                print('try again')
                break
                return
            bestgoalnode = bestgoalnode.parent
            for pxs,pys in zip(bestgoalnode.px,bestgoalnode.py):
                px.append(pxs)
                py.append(pys)
        #print(px,py)
        self.graph.add_edge(px,py)






if __name__ == '__main__':

    start = [4,3]
    goal = [8,8]
    grrt = GRRT(start,goal)
    grrt.planning()

    grrt.graph.drawgraph()


    #graph = Graph()
    #print(graph.px)
    #px = [0,1]
    #py = [3,1]
    #graph.add_edge(px,py)
    #graph.drawgraph()






