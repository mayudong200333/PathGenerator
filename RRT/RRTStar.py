## Path Planning Code that use RRT*
import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.cost = 0.0
        self.parent = None


class RRTStar:

    def __init__(self,start,goal,obstacleList,randArea,expanDis=0.5,goalSampleRate=20,maxIter=500):
        self.start = Node(start[0],start[1])
        self.end = Node(goal[0],goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expanDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.nodeList = []
        self.obstacleList = obstacleList

    def Planning(self,animation=True):
        animation = False

        self.nodeList.append(self.start)
        for i in range(self.maxIter):
            rnd = self.sample()
            minind = self.NearestIndex(rnd)
            newNode = self.steer(rnd,minind)
            if self.CollisionCheck(newNode):
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
            rnd = [random.uniform(self.minrand,self.maxrand),random.uniform(self.minrand,self.maxrand)]
        else:
            rnd = [self.end.x,self.end.y]
        return rnd

    def NearestIndex(self,rnd):
        # user can difine cost-function here
        # the cost function used here is 2-norm
        Jlist = [(node.x - rnd[0])**2 + (node.y - rnd[1])**2 for node in self.nodeList]
        minind = Jlist.index(min(Jlist))
        return minind

    def steer(self,rnd,minind):
        nearestNode = self.nodeList[minind]
        theta = math.atan2(rnd[1] - nearestNode.y,rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = minind
        return newNode

    def CollisionCheck(self,node):
        # check the point if in the obstacle
        for (ox,oy,size) in self.obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx*dx + dy*dy
            if d <= size **2:
                return False
        return True

    def CollisionCheck_extend(self,nearNode,theta,d):
        # check the line if in the obstacle
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d/self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.CollisionCheck(tmpNode):
                return False

        return True



    def find_near_nodes(self,newNode):
        nnode = len(self.nodeList)
        # r = self.expanDis * 5.0
        r = 50.0 * math.sqrt((math.log(nnode)/nnode))
        Jlist = [(node.x - newNode.x)**2 + (node.y - newNode.y)**2 for node in self.nodeList]
        nearinds = [Jlist.index(i) for i in Jlist if i <= r**2]
        return nearinds



    def choose_parent(self,newNode,nearinds):
        if len(nearinds) == 0:
            return newNode

        Jlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy,dx)
            if self.CollisionCheck_extend(self.nodeList[i],theta,d):
                Jlist.append(self.nodeList[i].cost+d)
            else:
                Jlist.append(float("inf"))

        mincost = min(Jlist)
        minind = nearinds[Jlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def rewire(self,newNode,nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]
            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy,dx)
                if self.CollisionCheck_extend(nearNode,theta,d):
                    nearNode.parent = nnode-1
                    nearNode.cost = scost

    def get_best_last_index(self):
        disglist = [self.calc_dist_to_goal(node.x,node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i
        return None

    def calc_dist_to_goal(self,x,y):
        return np.linalg.norm([x -self.end.x,y-self.end.y])

    def gen_final_path(self,goalind):
        path = [[self.end.x,self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x ,node.y])
            goalind = node.parent
        path.append([self.start.x,self.start.y])
        return path

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        import matplotlib.pyplot as plt
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

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
    rrt = RRTStar(start=[0, 0], goal=[10, 10],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=True)

    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()






















