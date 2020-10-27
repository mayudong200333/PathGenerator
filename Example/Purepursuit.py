import math
import os
import random
import sys
import matplotlib.pyplot as plt
import numpy as np

# 要修正、まだx軸沿った直線しか対応できていない

class Purepursuit:
    def __init__(self,ref_path,vref,state_now,L=2.885,vch=20.0,Ta=0.2,Td=0.05,a_accel=1.0,a_decel=2.5,v_max=3,a_init=0,Kp=1,Ki=0.04):
        # ref_path should be a straight line
        # state_now = [x,y,theta]
        self.ref_path = ref_path
        self.vref = vref
        self.state_now = state_now
        self.a_init = a_init
        self.path_grad = math.atan((refpath[0][0] - ref_path[-1][0])/(refpath[0][1]-ref_path[-1][1]))
        self.path_intercept = -self.path_grad*ref_path[0][0] +  ref_path[0][1]

        self.L = L
        self.vch = vch
        self.Ta = Ta
        self.Td = Td
        self.a_accel = a_accel
        self.a_decel = a_decel
        self.v_max = v_max
        self.Kp = Kp
        self.Ki = Ki

        self.dt = 0.05
        self.ei = []

        self.px = [state_now[0],]
        self.py = [state_now[1],]

        self.vcmdlist = []
        self.vlog = [self.vref,]

    def planning(self):
        vcmdlist = self.generate_vcmd()
        self.vcmdlist = vcmdlist
        v = self.vref
        x = self.state_now[0]
        y = self.state_now[1]
        a = self.a_init
        theta = self.state_now[2]
        Lfw = self.choice_Lfw(v)
        gamma = 0
        for i in range(1,len(self.ref_path)):
            ref_path_point = self.ref_path[i]
            vcmd = vcmdlist[i]
            Lfw = self.choice_Lfw(vcmd)
            gammac = self.steer_controller(Lfw,x,y,theta)
            print(gammac)
            ac = self.speed_controller(vcmd,v)
            dx,dy,dtheta,dgamma,dv,da = self.model([v,theta,gamma,a],[gammac,ac])

            x += dx*self.dt
            y += dy*self.dt
            self.px.append(x)
            self.py.append(y)

            theta += dtheta * self.dt
            gamma += dgamma * self.dt
            v += dv * self.dt
            a += da * self.dt

            self.vlog.append(v)
        a = np.cos(self.path_grad)
        b = np.sin(self.path_grad)
        self.px2 = math.cos(self.path_grad)*np.array(self.px)-math.sin(self.path_grad)*np.array(self.py)
        self.py2 = math.sin(self.path_grad)*np.array(self.px)+math.cos(self.path_grad)*np.array(self.py)

    # get from ref
    def choice_Lfw(self,vcmd):
        if vcmd < 5.36:
            Lfw = 3
            return Lfw
        elif vcmd >= 1.34 and vcmd < 5.36:
            Lfw = 2.24 * vcmd
            return Lfw
        else:
            Lfw = 12
            return Lfw

    # defined by mayudong
    def get_Lfw(self,x,y):
        target_x = self.ref_path[-1][0]
        target_y = self.ref_path[-1][1]
        Lfw = np.sqrt((x-target_x)**2+(y-target_y)**2)
        return Lfw

    def generate_vcmd(self):
        n = len(self.ref_path)
        vcmdlist = np.zeros(n)
        naccel = int((self.v_max-self.vref)/(self.a_accel*self.dt))
        stop_point = self.vref + (naccel - 1) * self.a_accel * self.dt
        print(stop_point)
        ndecel = int((stop_point-0)/(self.a_decel*self.dt))
        nconst = n-ndecel
        print(naccel)
        print(ndecel)
        print(nconst)
        vcmdlist[:naccel] = np.linspace(self.vref,stop_point,naccel)
        vcmdlist[naccel:nconst] = np.ones_like(vcmdlist[naccel:nconst])*stop_point
        vcmdlist[nconst:] = np.linspace(stop_point,0,ndecel)
        return vcmdlist


    def steer_controller(self,Lfw,x,y,theta):
        #center_y = (self.path_grad*x-y+self.path_intercept)/math.sqrt(self.path_grad**2+1)
        #theta2 = theta - math.atan(self.path_grad)
        center_y = y
        #a=center_y/Lfw
        ita = theta + math.asin(center_y/Lfw)
        gammac = -math.atan(self.L*math.sin(ita)/(Lfw/2+self.L/2*math.cos(ita)))
        return gammac


    def speed_controller(self,vcmd,v):
        e = vcmd - v
        self.ei.append(e)
        u = self.Kp * e + self.Ki * np.sum(e) * self.dt
        return u

    def model(self,state,input):
        # state = [v,theta,gamma,a]
        # input = [gammac,ac]
        v = state[0]
        theta = state[1]
        gamma = state[2]
        a = state[3]
        gammac = input[0]
        ac = input[1]
        Gss = 1 / (1 + (v / self.vch) ** 2)

        dx = v * math.cos(theta)
        dy = v * math.sin(theta)
        dtheta = v / self.L * math.tan(gamma) * Gss
        dgamma = 1/self.Td*(gammac-gamma)
        dv = a
        da = 1/self.Ta * (ac-a)
        return dx,dy,dtheta,dgamma,dv,da


if __name__ == '__main__':
    print('start')
    n = 150
    refpath_x = np.linspace(0,10,n)
    #refpath_y = np.linspace(0,5,n)
    refpath_y = np.zeros(n)
    t = np.linspace(0,0.05*200,n)
    refpath = np.vstack((refpath_x,refpath_y))
    refpath = refpath.T
    print(refpath[-1][0])
    print(refpath[-1][1])

    vref = 0
    state_now = [0,-3,0]

    pure = Purepursuit(refpath,vref,state_now)
    pure.planning()


    plt.figure()
    plt.plot(pure.px,pure.py)
    plt.plot(refpath_x,refpath_y)
    plt.figure()
    plt.plot(t,pure.vcmdlist)
    plt.plot(t,pure.vlog)
    plt.show()