#MARRTStar
#made by mayudong

from RRT.GRRT import Graph,Node
import copy
import random
import math
import numpy as np
import functools
import operator
import matplotlib.pyplot as plt
import matplotlib.patches as pat

class MARRTStar():
    def __init__(self,startlis,goallis,randArea=[0,9],obstacleList=[(2,4),(2.5,4),(3,3),(3,3.5),(3,4.5),(3,5),(3,4),(3.5,4),(4,4),(4.5,4),(5,4),(5,3.5),(5,3),(5,2.5),(5,2)],goalSampleRate=20,maxIter=100,cmax=6,maxspeedi=[0.5,0.5]):

        self.graph = Graph(size=(19, 19), xrange=randArea, yrange=randArea, obstacleList=obstacleList)
        self.graphnode = self.graph.nodeList

        self.agent_number = len(startlis)
        self.maxspeedi = maxspeedi
        if len(maxspeedi) != self.agent_number:
            print('You should redefine the maxspeedi')

        self.startlis = []
        self.goallis = []
        self.get_start_goal(startlis,goallis)

        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.cmax = cmax

        self.nodeList = [[start,] for start in self.startlis]


        px = [start.x for start in self.startlis]
        py = [start.y for start in self.startlis]

        self.graph.add_edge(px,py)


    def get_start_goal(self,startlis,goallis):
        for start in startlis:
            for node in self.graphnode:
                if len(self.startlis) == self.agent_number:
                    break
                if node.x == start[0] and node.y == start[1]:
                    self.startlis.append(copy.deepcopy(node))
        for goal in goallis:
            for node in self.graphnode:
                if len(self.goallis) == self.agent_number:
                    break
                if node.x == goal[0] and node.y == goal[1]:
                    self.goallis.append(copy.deepcopy(node))

        if len(self.startlis) != len(self.goallis):
            print('You should redefine the start or goal region')

    def planning(self):
        print('start planning')
        for i in range(self.maxIter):
            print(i)
            rndlis = self.sample()
            xnearestlis = self.nearest(rndlis)
            #print(xnearestlis[0])
            #print(xnearestlis[0].x,xnearestlis[0].y)
            #print(xnearestlis[0].px,xnearestlis[0].py)
            #print(rndlis[0].x, rndlis[0].y)
            xnewlis,pnewlis,c = self.greedy(xnearestlis,rndlis)
            #print(xnewlis[0])
            #print(xnewlis[0].x, xnewlis[0].y)
            #print(xnewlis[0].px, xnewlis[0].py)
            #check = [xnew==xnearest for xnew,xnearest in zip(xnewlis,xnearestlis)]
            if self.calc_dis(xnewlis,xnearestlis) != 0:
                for j in range(self.agent_number):
                    xnewlis[j].parent = xnearestlis[j]
                    self.nodeList[j].append(xnewlis[j])

                #xminlis = xnearestlis

                '''
                xnearlis = self.near(xnewlis)
                #print(xnewlis[0].x, xnewlis[0].y)


                xminendlis = None
                for xnears in xnearlis:
                    #print(xnears[0].x, xnears[0].y)
                    x_lis,p_lis,costpre = self.greedy(xnears,xnewlis)
                    #print(xnears[0].x, xnears[0].y)
                    if self.calc_dis(x_lis,xnewlis) == 0:
                        c_ = self.calc_cost(xnears) + costpre
                        #print(c_,self.calc_cost(xnears) + costpre,self.calc_cost(xnewlis),)
                        if c_ < self.calc_cost(xnewlis):
                            #print('b')
                            xminlis = xnears
                            xminendlis = x_lis

                #print(xminlis[0].x,xminlis[0].y)
                #print(xnewlis[0].x, xnewlis[0].y)
                for k in range(self.agent_number):
                    self.nodeList[k][self.nodeList[k].index(xnewlis[k])].parent = xminlis[k]
                    if xminendlis:
                        self.nodeList[k][self.nodeList[k].index(xnewlis[k])].px = xminendlis[k].px
                        self.nodeList[k][self.nodeList[k].index(xnewlis[k])].py = xminendlis[k].py
                        self.nodeList[k][self.nodeList[k].index(xnewlis[k])].cost = xminendlis[k].cost

                try:
                    xnearlis.pop(xnearlis.index(xminlis))
                except:
                    continue

                for xnears in xnearlis:
                    x__lis,p__lis,costpre = self.greedy(xnewlis,xnears)
                    #print(self.calc_cost(xnears) , self.calc_cost(x__lis))
                    if self.calc_dis(x__lis,xnears) == 0 and self.calc_cost(xnears) > self.calc_cost(x__lis):
                        for l in range(self.agent_number):
                            #print('a')
                            self.nodeList[l][self.nodeList.index(xnears[l])].parent = xnewlis[l]
                            self.nodeList[l][self.nodeList.index(xnears[l])].px = x__lis[l].px
                            self.nodeList[l][self.nodeList.index(xnears)[l]].py = x__lis[l].py
                            self.nodeList[l][self.nodeList.index(xnears)[l]].cost = x__lis[l].cost
                '''

        print('end')
        self.generate_path()



    def sample(self):
        rndlis = []
        for i in range(self.agent_number):
            if random.randint(0, 100) > self.goalSampleRate:
                s = self.graphnode[random.randint(0,len(self.graphnode)-1)]
                rnd = Node(s.x,s.y)
                rnd.cost = 0
                rnd.children = s.children
            else:
                s = self.goallis[i]
                rnd = Node(s.x,s.y)
                rnd.cost = 0
                rnd.children = s.children
            rndlis.append(rnd)
        return rndlis

    def nearest(self,rndlis):
        xnearestlis = []
        k = float('inf')
        for i in range(self.agent_number):
            Jlist = [self.cost(rndlis[i],node) for node in self.nodeList[i]]
            neari = Jlist.index(min(Jlist))
            xnearestlis.append(self.nodeList[i][neari])
        return xnearestlis

    def cost(self,node1,node2):
        cost = 0
        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y
        speedi = self.maxspeedi[0]
        cost += math.sqrt((x1-x2)**2+(y1-y2)**2)/speedi
        return cost

    def greedy(self,slis,dlis):
        #print(slis[1].x,slis[1].y)
        #print(dlis[0].x,dlis[0].y)
        j = 0
        xlisout = []
        c = 0
        pathlis = [[] for i in range(self.agent_number)]

        for x in slis:
            sx = Node(x.x, x.y)
            sx.cost = x.cost
            sx.children = x.children
            xlisout.append(sx)
            j += 1
        pxlis = [[] for i in range(self.agent_number)]
        pylis = [[] for i in range(self.agent_number)]
        costlis = np.zeros(self.agent_number)

        while c<=self.cmax:
            if self.calc_dis(xlisout,dlis) == 0:
                break
            xlispre = [[] for i in range(self.agent_number)]
            #xlispre = []
            costpre = np.zeros(self.agent_number)
            i = 0
            for x in xlisout:
                #if x.x == dlis[i].x and x.y == dlis[i].y:
                    #i += 1
                    #continue
                childrenDisLis = [self.hueristic(child,dlis[i]) for child in x.children]
                x_ = x.children[childrenDisLis.index(min(childrenDisLis))]

                cx_ = Node(x_.x,x_.y)
                cx_.children = x_.children

                costpre[i] = self.cost(x,cx_)
                c += self.cost(x,cx_)
                costlis[i] += self.cost(x,cx_)
                #print(costpre)
                xlispre[i] = xlisout[i]
                pathlis[i].append(cx_)  # Is it necessary?
                pxlis[i].append(cx_.x)
                pylis[i].append(cx_.y)
                xlisout[i] = cx_
                i += 1
            if not self.CollisionFree(pathlis):
                print('Not free')

                j = 0
                for x in xlispre:
                    x.px = pxlis[j][:-1]
                    x.py = pylis[j][:-1]
                    x.cost = slis[j].cost + costlis[j] - costpre[j]
                    pathlis[j].pop(-1)
                    j +=1
                c = sum(costlis)

                return xlispre,pathlis,c



        j = 0
        for x in xlisout:
            x.px = pxlis[j]
            x.py = pylis[j]
            x.cost = slis[j].cost + costlis[j]
            j += 1

        return xlisout,pathlis,c

    def hueristic(self,node1,node2):
        # you can difine your own hueristic here
        # but now I use L2-norm defined in graph caluc_dis
        dis = self.graph.calc_node_dis(node1,node2)
        return dis

    def CollisionFree(self,pathlis):
        for i in range(len(pathlis[0])):
            pxynow = [[] for n in range(self.agent_number)]
            centerlis = [[] for n in range(self.agent_number)]
            for j in range(self.agent_number):
                agentiw = [pathlis[j][i].x,pathlis[j][i].y]
                if agentiw in pxynow: # you can change here to set distance
                    return False
                pxynow[j] = agentiw
                if i >= 1:
                    centerpoint = [(pathlis[j][i].x+pathlis[j][i-1].x)/2,(pathlis[j][i].y+pathlis[j][i-1].y)/2]
                    if centerpoint in centerlis:
                        return False
                    centerlis[j] = centerpoint
        return True

    def near(self,newnodelis):
        nearNodesLis = [[] for n in range(self.agent_number)]
        nnodelis = [len(self.nodeList[i]) for i in range(self.agent_number)]
        rlis = [max(10.0 * math.sqrt((math.log(nnode) / nnode)),4) for nnode in nnodelis]
        for i in range(self.agent_number):
            Jlist = [self.cost(newnodelis[i],node) for node in self.nodeList[i]]
            #print(Jlist)
            nearindsLis = [Jlist.index(j) for j in Jlist if j <= rlis[i] ** 2 and j != 0]
            nearindsLis = set(nearindsLis)
            for nearind in nearindsLis:
                nearNodesLis[i].append(self.nodeList[i][nearind])
        nearNodesLis = self.findind(nearNodesLis)
        return nearNodesLis

    def generate_path(self):
        px = [[] for n in range(self.agent_number)]
        py = [[] for n in range(self.agent_number)]
        px_draw = []
        py_draw = []

        goalinds = []
        for i in range(self.agent_number):
            j = 0
            for node in self.nodeList[i]:
                if node.parent:
                    if node.x == self.goallis[i].x and node.y == self.goallis[i].y:
                        goalinds.append(j)
                        break
                j += 1
        if len(goalinds) != self.agent_number:
            print('You should try to increase iteration number')
            return
        i = 0
        for goalind in goalinds:
            goalnode = self.nodeList[i][goalind]
            for pxs,pys in zip(goalnode.px,goalnode.py):
                px[i].append(pxs)
                py[i].append(pys)
                px_draw.append(pxs)
                py_draw.append(pys)
            while goalnode.parent is not None:
                print(goalnode.x,goalnode.y,'parent',goalnode.parent.x,goalnode.parent.y)
                if goalnode.x == goalnode.parent.x and goalnode.y == goalnode.parent.y:
                    return
                goalnode = goalnode.parent
                for pxs, pys in zip(goalnode.px, goalnode.py):
                    #print('generating now')
                    px[i].append(pxs)
                    py[i].append(pys)
                    px_draw.append(pxs)
                    py_draw.append(pys)
            i += 1

        self.graph.add_edge(px_draw,py_draw)

    def findind(self,nodelis):
        lenlis = [len(al) for al in nodelis]
        seki = functools.reduce(operator.mul, lenlis)
        re = [[] for nu in range(seki)]
        j = 0
        k = len(nodelis[0])
        for al in nodelis:
            i = 0
            for number in range(k):
                for n in al:
                    rel = re[i*int(seki/k):(i+1)*int(seki/k)]
                    for rem in rel:
                        rem.append(n)
                    i += 1
            j += 1
            if j < len(nodelis):
                k *= len(nodelis[j])
        return re

    def calc_dis(self,nodes1,nodes2):
        d = 0
        for i in range(self.agent_number):
            d += (nodes1[i].x-nodes2[i].x)**2 + (nodes1[i].y-nodes2[i].y)**2
        return d

    def calc_cost(self,nodes):
        c = 0
        for node in nodes:
            c += node.cost

        return c






if __name__ == '__main__':

    start = [(4,3),(0,0),[1,1]]
    goal = [(8,8),(7,8),[0,5]]
    marrt = MARRTStar(start,goal)

    marrt.planning()
    marrt.graph.drawgraph()
    #print(marrt.graph.graphMatrix)
    for i in range(marrt.agent_number):
        print('agent',i+1)
        for node in marrt.nodeList[i]:
            print(node.px, node.py)
            if node.parent is not None:
                print(node.x,node.y,'parent',node.parent.x,node.parent.y)
                print(node.cost)



    '''
    a = [[1,2,3],[2,3,6,10],[9,8,7],[1,2]]
    lenalis = [len(al) for al in a]
    seki = functools.reduce(operator.mul,lenalis)
    re = [[] for n in range(seki)]
    j = 0
    k = len(a[0])
    for al in a:
        i = 0
        for number in range(k):
            for n in al:
                rel = re[i*int(seki/k):(i+1)*int(seki/k)]
                for rem in rel:
                    rem.append(n)
                i += 1
        j += 1
        if j < len(a):
            k *= len(a[j])


    print(re)

    '''

    '''
    check for function CollisionFree
    Node1 = Node(1,1)
    Node2 = Node(1,2)
    Node3 = Node(1,2)
    Node4 = Node(2,2)
    agent1path = [Node1,Node2]
    agent2path = [Node3,Node4]
    pathlis = [agent1path,agent2path]
    print(marrt.CollisionFree(pathlis))
    '''

