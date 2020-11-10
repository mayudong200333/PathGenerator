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
    def __init__(self,startlis,goallis,randArea=[0,9],obstacleList=[(2,4),(2.5,4),(3,3),(3,3.5),(3,4.5),(3,5),(3,4),(3.5,4),(4,4),(4.5,4),(5,4),(5,3.5),(5,3),(5,2.5),(5,2)],goalSampleRate=20,maxIter=60,cmax=4,maxspeedi=[0.5,0.5]):

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

        self.pathx = [[] for n in range(self.agent_number)]
        self.pathy = [[] for n in range(self.agent_number)]


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
            xnewlis,pnewlis,c = self.greedy(xnearestlis,rndlis)

            if self.calc_dis(xnewlis,xnearestlis) != 0:

                for j in range(self.agent_number):
                    #xnewlis[j].parent = xnearestlis[j]
                    self.nodeList[j].append(xnewlis[j])

                xminlis = xnearestlis


                xnearlis = self.near(xnewlis)
                #print(xnewlis[0].x, xnewlis[0].y)

                xminendlis = None
                for xnears in xnearlis:
                    x_lis,p_lis,costpre = self.greedy(xnears,xnewlis)
                    if self.calc_dis(x_lis,xnewlis) == 0:
                        c_ = self.calc_cost(xnears) + costpre
                        if c_ < self.calc_cost(xnewlis):
                            #print('b')
                            xminlis = xnears
                            xminendlis = x_lis

                for k in range(self.agent_number):
                    indk = self.nodeList[k].index(xnewlis[k])
                    self.nodeList[k][indk].parent = xminlis[k]
                    if xminendlis:
                        self.nodeList[k][indk].px = xminendlis[k].px
                        self.nodeList[k][indk].py = xminendlis[k].py
                        self.nodeList[k][indk].cost = xminendlis[k].cost

                try:
                    xnearlis.pop(xnearlis.index(xminlis))
                except:
                    pass

                for xnears in xnearlis:
                    x__lis,p__lis,costpre = self.greedy(xnewlis,xnears)
                    #print(self.calc_cost(xnears) , self.calc_cost(x__lis))
                    if self.calc_dis(x__lis,xnears) == 0 and self.calc_cost(xnears) > self.calc_cost(x__lis):
                        for l in range(self.agent_number):
                            #print('a')
                            indl = self.nodeList[l].index(xnears[l])
                            self.nodeList[l][indl].parent = xnewlis[l]
                            self.nodeList[l][indl].px = x__lis[l].px
                            self.nodeList[l][indl].py = x__lis[l].py
                            self.nodeList[l][indl].cost = x__lis[l].cost



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
                if x.x == dlis[i].x and x.y == dlis[i].y:
                    xlispre[i] = xlisout[i]
                    costpre[i] = 0
                    pathlis[i].append(x)
                    pxlis[i].append(x.x)
                    pylis[i].append(x.y)
                    i += 1
                    continue
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
            '''
            if not self.CollisionFree(pathlis):
                #print('Not free')

                j = 0
                for x in xlispre:
                    x.px = pxlis[j][:-1]
                    x.py = pylis[j][:-1]
                    x.cost = slis[j].cost + costlis[j] - costpre[j]
                    pathlis[j].pop(-1)
                    j +=1
                c = sum(costlis)

                return xlispre,pathlis,c
            '''



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
        rlis = [max(10.0 * math.sqrt((math.log(nnode) / nnode)),3) for nnode in nnodelis]
        for i in range(self.agent_number):
            Jlist = [self.cost(newnodelis[i],node) for node in self.nodeList[i]]
            nearindsLis = []
            ind = 0
            for j in Jlist:
                if j <= rlis[i]**2 and j!=0:
                    nearindsLis.append(ind)
                ind += 1
            #nearindsLis = [Jlist.index(j) for j in Jlist if j <= rlis[i] ** 2 and j != 0]
            for nearind in nearindsLis:
                nearNodesLis[i].append(self.nodeList[i][nearind])
        nearNodesLis = self.findind(nearNodesLis)
        return nearNodesLis

    def generate_path(self):
        px = [[] for n in range(self.agent_number)]
        py = [[] for n in range(self.agent_number)]
        px_draw = []
        py_draw = []

        goalinds = [[] for n in range(self.agent_number)]
        goalcosts = [float('inf') for n in range(self.agent_number)]
        for i in range(self.agent_number):
            j = 0
            for node in self.nodeList[i]:
                # if self.calc_simple_dis2(node,self.goallis[i]) <= 1:
                if node.x == self.goallis[i].x and node.y == self.goallis[i].y:
                    if node.cost < goalcosts[i]:
                        goalinds[i] = j
                        goalcosts[i] = node.cost
                j += 1
        print(goalinds)
        if len(goalinds) != self.agent_number:
            print('You should try to increase iteration number')
            return

        #goalinds = [goalind[-1] for goalind in goalinds]
        #print(goalinds)
        i = 0
        for goalind in goalinds:
            goalnode = self.nodeList[i][goalind]
            goalnode.px.reverse()
            goalnode.py.reverse()
            for pxs,pys in zip(goalnode.px,goalnode.py):
                px[i].append(pxs)
                py[i].append(pys)
                px_draw.append(pxs)
                py_draw.append(pys)
            while goalnode.parent is not None:
                print(goalnode.x,goalnode.y,'parent',goalnode.parent.x,goalnode.parent.y)
                goalnode = goalnode.parent
                goalnode.px.reverse()
                goalnode.py.reverse()
                for pxs, pys in zip(goalnode.px, goalnode.py):
                    #print('generating now')
                    px[i].append(pxs)
                    py[i].append(pys)
                    px_draw.append(pxs)
                    py_draw.append(pys)
            i += 1
            
        #self.graph.add_edge(px_draw,py_draw)
        for i in range(self.agent_number):
            px[i].reverse()
            py[i].reverse()
            self.pathx[i] = px[i]
            self.pathy[i] = py[i]
            


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
    def calc_simple_dis2(self,node1,node2):
        return (node1.x-node2.x)**2 + (node1.y-node2.y)**2






if __name__ == '__main__':


    obstacleList=[(2, 4), (2.5, 4),(2,4.5),(2.5,4.5),(2.5,5),(2,5),(4.5,5),(4.5,5.5),(5,5),(5.5,5),(5,5.5),(5.5,5.5),(2,6),(2.5,6),(2,6.5),(2.5,6.5),(7,4),(7.5,4),(7,4.5),(7.5,4.5),(6,7),(6.5,7.5),(6,7.5),(6.5,7)]
    #start = [(8,1),(1,7),(1,1),(4,1),(8.5,1)]
    #goal = [(1,8),(6,1),(7,8),(4,8),(5.5,8)]
    start = [(4, 1),  [4, 8]]
    goal = [(4, 8),  [4, 1]]
    marrt = MARRTStar(start,goal,obstacleList=obstacleList)

    marrt.planning()
    marrt.graph.drawgraph()
    pathx = marrt.pathx
    pathy = marrt.pathy
    print(pathx)
    print(pathy)
    #for i in range(marrt.agent_number):
        #for j in range(len(pathx[0])):
            #if j+1 > len(pathx[0])-1:
                #continue
            #plt.plot([pathx[i][j],pathx[i][j+1]],[pathy[i][j],pathy[i][j+1]],"-g")
            #plt.pause(0.01)
    plt.xlim(-2, 10)
    plt.ylim(-2, 10)

    a = marrt.pathx
    b = marrt.pathy

    a0 = a[0]
    b0 = b[0]
    a1 = a[1]
    b1 = b[1]
    #a2 = a[2]
    #b2 = b[2]
    #a3 = a[3]
    #b3 = b[3]
    for i in range(max(len(a0),len(a1))):
        try:
            plt.plot([a0[i],a0[i+1]],[b0[i],b0[i+1]],color='red')
        except:
            pass
        try:
            plt.plot([a1[i], a1[i + 1]], [b1[i], b1[i + 1]],color='gray')
        except:
            pass
        #try:
            #plt.plot([a2[i], a2[i + 1]], [b2[i], b2[i + 1]],color='black')
        #except:
            #pass
        #try:
            #plt.plot([a3[i], a3[i + 1]], [b3[i], b3[i + 1]],color='blue')
        #except:
            #pass
        plt.pause(1)
    plt.show()




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

