#MARRTStar
#a new algorithm made by mayudong

from RRT.GRRT import Graph,Node,GRRT
import sys
import copy
import random
import math
import numpy as np
import time
import functools
import operator
import matplotlib.pyplot as plt
import matplotlib.animation as animation



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
        self.edge_length = self.randArea[1]/(self.size[0]-1)

    def planning(self):
        start = time.time()
        #ob_first = copy.copy(self.obstacleList)
        snew,gnew = self.taskAssignment()
        self.startlis = snew
        self.goallis = gnew
        #print(ob_first)
        first_agent = GRRT(snew[0],gnew[0],obstacleList=self.obstacleList,size=self.size,xrange=self.randArea,yrange=self.randArea,goalSampleRate=self.goalSampleRate,maxIter=self.maxIter,cmax=self.cmax,animation=self.animation)
        first_agent.planning()
        first_agent_path = [first_agent.px,first_agent.py]
        self.agentPathList_last.append(copy.deepcopy(first_agent_path))
        self.agentPathList.append(first_agent_path)
        for i in range(1,self.n_agent):
            #obi = copy.copy(self.obstacleList)
            #self.obstacleList.append(gnew[i])
            #print(self.startlis[i],self.goallis[i])
            #print(snew[i])
            #print(gnew[i])
            agenti = GRRT(snew[i],gnew[i],obstacleList=self.obstacleList,size=self.size,xrange=self.randArea,yrange=self.randArea,agentpath=self.agentPathList,goalSampleRate=self.goalSampleRate,maxIter=self.maxIter,cmax=self.cmax,animation=self.animation)
            agenti.int_planning()
            agenti_path = [agenti.px,agenti.py]
            self.agentPathList_last.append(copy.deepcopy(agenti_path))
            self.agentPathList.append(agenti_path)
        self.elapsed_time = time.time() - start

    def taskAssignment(self):
        costlis = []
        snew = []
        gnew = []
        #i = 0
        for s, d in zip(self.startlis, self.goallis):
            s = np.array(s)
            d = np.array(d)
            ecost = np.linalg.norm(s - d, ord=1)
            costlis.append(ecost)
        ind = np.argsort(costlis)
        for inx in ind:
            snew.append(self.startlis[inx])
            gnew.append(self.goallis[inx])
        return snew, gnew

    def calc_cost(self):
        opt_cost = 0
        real_cost = 0
        for s,d in zip(self.startlis,self.goallis):
            s = np.array(s)
            d = np.array(d)
            ecost = np.linalg.norm(s-d,ord=1)
            opt_cost+=ecost

        for i in range(self.n_agent):
            leni = len(self.agentPathList_last[i][0])
            #print(self.agentPathList_last[i][0])
            real_cost += self.edge_length * (leni-1)

        per = 1 - (real_cost-opt_cost)/opt_cost

        return opt_cost,real_cost,per

    def draw_animation(self,namelabel=2):
        pathlist = self.agentPathList
        lenmax = 0
        colorlis = ['red','gray','black','blue','purple','pink','yellow','brown','aqua','gold']
        i = 0
        for s,d in zip(self.startlis,self.goallis):
            plt.scatter(s[0],s[1],marker="D",color=colorlis[i],label='agent'+str(i))
            plt.scatter(d[0],d[1], marker="*", color=colorlis[i])
            i += 1
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0, fontsize=10)
        for i in range(self.n_agent):
            ai = pathlist[i][0]
            if len(ai) > lenmax:
                lenmax = len(ai)
        for i in range(lenmax):
            for j in range(self.n_agent):
                aj = pathlist[j][0]
                bj = pathlist[j][1]
                try:
                    plt.plot([aj[i], aj[i + 1]], [bj[i], bj[i + 1]], color=colorlis[j])
                except:
                    pass
            plt.pause(1)

        filename = "agent"+str(self.n_agent)+"_"+str(namelabel)+".png"
        plt.savefig("agent"+str(self.n_agent)+'/'+filename, bbox_inches='tight')
        plt.show()

def random_sample_test(n,agent_number,obstacleList):
    perlis = []
    timelis = []
    for j in range(n):
        obstacleListg = copy.deepcopy(obstacleList)
        graph = Graph(obstacleList=obstacleListg)
        graphnodelis = graph.nodeList
        start = []
        goal = []
        for i in range(agent_number):
            si = graphnodelis[random.randint(0, len(graphnodelis) - 1)]
            graphnodelis.pop(graphnodelis.index(si))
            start.append((si.x, si.y))
            di = graphnodelis[random.randint(0, len(graphnodelis) - 1)]
            graphnodelis.pop(graphnodelis.index(di))
            goal.append((di.x, di.y))
        graph2 = Graph(obstacleList=obstacleListg)
        graph2.drawgraph()
        marrt = MAGRRT(start, goal, obstacleList=obstacleListg)
        marrt.planning()
        opt, real, per = marrt.calc_cost()
        marrt.draw_animation(namelabel=j)
        timei = marrt.elapsed_time
        perlis.append(per)
        timelis.append(timei)
    perfilename = "agent"+str(agent_number)+'/'+"per_"+str(agent_number)
    timefilename = "agent"+str(agent_number)+'/'+"time_"+str(agent_number)
    np.save(perfilename,perlis)
    np.save(timefilename,timelis)









if __name__ == '__main__':

    start = [(1, 5.5), (3.5, 5.5)]
    goal = [(3.5, 5.5), (1, 5.5)]
    #start = [(4, 1), (4, 8)]
    #goal = [(4, 9), (4, 5)]
    #start = [(8, 1), (1, 7), (1, 1), (4, 1), (8.5, 1)]
    #goal = [(1,8),(6,1),(7,8),(4,8),(5.5,8)]
    #start = [(0,0),(4, 1), (8.5, 1), (1, 7), (1, 1), (8, 1)]
    #goal = [(9,9),(4,8),(5.5,8),(6,1),(7,8),(1,8)]

    #start = [(4.5,0),(8,0),(1.5,4.5),(5,6.5),(0,1.5),(2,0)]
    #goal = [(5,0),(6,2),(4,3),(3,1),(8,2),(8,8.5)]


    obstacleList = [(2, 4), (2.5, 4), (2, 4.5), (2.5, 4.5), (2.5, 5), (2, 5), (4.5, 5), (4.5, 5.5), (5, 5), (5.5, 5),
                    (5, 5.5), (5.5, 5.5), (2, 6), (2.5, 6), (2, 6.5), (2.5, 6.5), (7, 4), (7.5, 4), (7, 4.5),
                    (7.5, 4.5), (6, 7), (6.5, 7.5), (6, 7.5), (6.5, 7),(4,2.5),(4.5,2.5),(4,2),(4.5,2)]

    '''
    agent_number = 5
    n = 1
    random_sample_test(n,agent_number,obstacleList)
    '''

    '''
    perlis = []
    timelis = []
    label = []
    agent_number = 10
    for i in range(2,agent_number+1):
        peri_path = 'agent'+str(i)+'/'+'per_'+str(i)+'.npy'
        timei_path = 'agent'+str(i)+'/'+'time_'+str(i)+'.npy'
        label.append(str(i))
        perlis.append(np.load(peri_path)*100)
        timelis.append(np.load(timei_path))
    print(perlis)
    fig, ax = plt.subplots()
    bp = ax.boxplot(perlis)
    ax.set_xticklabels(label)
    plt.xlabel('agent number')
    plt.ylabel('performance(%)')
    plt.ylim([70, 102])
    plt.grid()
    plt.show()
    '''





    graph = Graph(obstacleList=obstacleList)
    graphnodelis = graph.nodeList
    graph.drawgraph()
    marrt = MAGRRT(start, goal, obstacleList=obstacleList)
    marrt.planning()
    opt,real,per=marrt.calc_cost()
    #marrt.draw_animation()
    print(marrt.elapsed_time)
    print(opt,real,per)




    pathlist = marrt.agentPathList_last


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




     
    for i in range(len(a1)):
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
            plt.plot([a5[i], a5[i + 1]], [b5[i], b5[i + 1]],color='pink')
        except:
            pass
        '''
        plt.pause(1)

    plt.show()





