import math
import matplotlib.pyplot as plt
class Dubin:
    """
    Dubins path planner
    start = [sx,sy,syaw]
    end = [ex,ey,syaw]
    output:
    px py pyaw mode
    """
    def __init__(self,start,end,c):
        self.sx = start[0]
        self.sy = start[1]
        self.syaw = start[2]
        self.ex = end[0]
        self.ey = end[1]
        self.eyaw = end[2]
        self.c = c
        self.planners = [self.LSL,self.RSR,self.LSR,self.RSL,self.RLR,self.LRL]
        self.d,self.alpha,self.beta = self.normalize_path()
        self.px,self.py,self.pyaw,self.length = self.planning()



    def planning(self):
        bcost = float("inf")
        bt, bp, bq, bmode = None, None, None, None
        for planner in self.planners:
            t,p,q,mode = planner(self.alpha,self.beta,self.d)
            if t is None:
                #  print("".join(mode) + " cannot generate path")
                continue

            cost = (abs(t) + abs(p) + abs(q))
            if bcost > cost:
                bt, bp, bq, bmode = t, p, q, mode
                bcost = cost

        px, py, pyaw = self.generate_course([bt, bp, bq], bmode)

        px2 = [math.cos(-self.syaw) * x + math.sin(-self.syaw) *
              y + self.sx for x, y in zip(px, py)]
        py2 = [- math.sin(-self.syaw) * x + math.cos(-self.syaw) *
              y + self.sy for x, y in zip(px, py)]
        pyaw2 = [self.pi_2_pi(iyaw + self.syaw) for iyaw in pyaw]

        return px2,py2,pyaw2,bcost
    def normalize_path(self):
        dx = self.ex - self.sx
        dy = self.ey - self.sy

        lex = math.cos(self.syaw) * dx + math.sin(self.syaw) * dy
        ley = - math.sin(self.syaw) * dx + math.cos(self.syaw) * dy
        leyaw = self.eyaw - self.syaw

        D = math.sqrt(dx**2 + dy**2)
        d = D/self.c
        dtheta = self.mod2pi(math.atan2(ley,lex))
        alpha = self.mod2pi(-dtheta)
        beta = self.mod2pi(leyaw-dtheta)
        return d,alpha,beta

    def generate_course(self,length,mode):
        px = [0.0]
        py = [0.0]
        pyaw = [0.0]

        for m, l in zip(mode, length):
            pd = 0.0
            if m is "S":
                d = 1.0 / self.c
            else:  # turning couse
                d = math.radians(3.0)

            while pd < abs(l - d):
                #  print(pd, l)
                px.append(px[-1] + d * self.c * math.cos(pyaw[-1]))
                py.append(py[-1] + d * self.c * math.sin(pyaw[-1]))

                if m is "L":  # left turn
                    pyaw.append(pyaw[-1] + d)
                elif m is "S":  # Straight
                    pyaw.append(pyaw[-1])
                elif m is "R":  # right turn
                    pyaw.append(pyaw[-1] - d)
                pd += d
            else:
                d = l - pd
                px.append(px[-1] + d * self.c * math.cos(pyaw[-1]))
                py.append(py[-1] + d * self.c * math.sin(pyaw[-1]))

                if m is "L":  # left turn
                    pyaw.append(pyaw[-1] + d)
                elif m is "S":  # Straight
                    pyaw.append(pyaw[-1])
                elif m is "R":  # right turn
                    pyaw.append(pyaw[-1] - d)
                pd += d
        return px,py,pyaw

    def mod2pi(self,theta):
        return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)

    def pi_2_pi(self,angle):
        while (angle >= math.pi):
            angle = angle - 2.0 * math.pi

        while (angle <= -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def LSL(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp0 = d + sa - sb

        mode = ["L", "S", "L"]
        p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
        if p_squared < 0:
            return None, None, None, mode
        tmp1 = math.atan2((cb - ca), tmp0)
        t = self.mod2pi(-alpha + tmp1)
        p = math.sqrt(p_squared)
        q = self.mod2pi(beta - tmp1)
        #  print(math.degrees(t), p, math.degrees(q))

        return t, p, q, mode

    def RSR(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        tmp0 = d - sa + sb
        mode = ["R", "S", "R"]
        p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
        if p_squared < 0:
            return None, None, None, mode
        tmp1 = math.atan2((ca - cb), tmp0)
        t = self.mod2pi(alpha - tmp1)
        p = math.sqrt(p_squared)
        q = self.mod2pi(-beta + tmp1)

        return t, p, q, mode

    def LSR(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
        mode = ["L", "S", "R"]
        if p_squared < 0:
            return None, None, None, mode
        p = math.sqrt(p_squared)
        tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
        t = self.mod2pi(-alpha + tmp2)
        q = self.mod2pi(-self.mod2pi(beta) + tmp2)

        return t, p, q, mode

    def RSL(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
        mode = ["R", "S", "L"]
        if p_squared < 0:
            return None, None, None, mode
        p = math.sqrt(p_squared)
        tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
        t = self.mod2pi(alpha - tmp2)
        q = self.mod2pi(beta - tmp2)

        return t, p, q, mode

    def RLR(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        mode = ["R", "L", "R"]
        tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
        if abs(tmp_rlr) > 1.0:
            return None, None, None, mode

        p = self.mod2pi(2 * math.pi - math.acos(tmp_rlr))
        t = self.mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + self.mod2pi(p / 2.0))
        q = self.mod2pi(alpha - beta - t + self.mod2pi(p))
        return t, p, q, mode

    def LRL(self,alpha, beta, d):
        sa = math.sin(alpha)
        sb = math.sin(beta)
        ca = math.cos(alpha)
        cb = math.cos(beta)
        c_ab = math.cos(alpha - beta)

        mode = ["L", "R", "L"]
        tmp_lrl = (6. - d * d + 2 * c_ab + 2 * d * (- sa + sb)) / 8.
        if abs(tmp_lrl) > 1:
            return None, None, None, mode
        p = self.mod2pi(2 * math.pi - math.acos(tmp_lrl))
        t = self.mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.)
        q = self.mod2pi(self.mod2pi(beta) - alpha - t + self.mod2pi(p))

        return t, p, q, mode




if __name__ == '__main__':
    print("start")
    start = [1,1,math.radians(45.0)]
    end = [-3,-5,math.radians(-45.0)]

    plan = Dubin(start,end,1.0)
    px = plan.px
    py = plan.py
    print(len(px))
    plt.plot(px, py)
    plt.grid(True)
    plt.axis("equal")
    plt.show()





