#!/usr/bin/python2.7

# import dubins
import dubins_interval.scripts.pydubins as pydubins
import math
from matplotlib import pyplot as plt
import numpy as np

class DubinsInterval():
    def __init__(self, R, qi, qf, a, b):
        self.best_path_cost = 10**9
        self.params = pydubins.Param(pydubins.Waypoint(0.0, 0.0, 0.0), 0, 0)
        self.params.seg_final = (0,0,0)
        
        # # to add: .append([values])
        # # at most will be five segments
        # self.best_path_segments = []

        # LSL = 0
        # LSR = 1
        # RSL = 2
        # RSR = 3
        # RLR = 4
        # LRL = 5
        self.best_path_type = int()

        self.path_types = ["LSL", "LSR", "RSL", "RSR", "RLR", "LRL"]

        self.R = R
        self.qi = qi
        self.qf = qf
        self.a = a
        self.b = b
        self.step = 0.1

        cases = ["1", "2", "3", "4", "5", "6", "7", "8", "9"]

        for parameter in cases:

            self.compute(parameter)

            # march through each possible interval case to select the best cost
            # conditions: (theta)i = al and (theta)f = bl
            # qi = (self.qi[0], self.qi[1], self.a[0])
            # qf = (self.qf[0], self.qf[1], self.b[0])
            # path = self.shortest_path(qi, qf, self.R)
            # self.update_path(path)
            # thing = path.path(qi, qf, self.R, self.step)

            

        # self.results()
        # self.draw_path()

    def compute(self, parameter):

        if (parameter == "1"):
            # conditions: (theta)i = al and (theta)f = bl
            qi = (self.qi[0], self.qi[1], self.a[0])
            qf = (self.qf[0], self.qf[1], self.b[0])
            self.shortest_path(qi, qf, self.R)

        elif (parameter == "2"):
            # conditions: (theta)i = al and bl < (theta)f < bh
            qi = (self.qi[0], self.qi[1], self.a[0])

            interval_step = 0
            for t in np.arange(self.b[0], self.b[1], self.step):
                interval_step = interval_step + t
                qf = (self.qf[0], self.qf[1], self.b[0]+interval_step)
                self.shortest_path(qi, qf, self.R)

        elif (parameter == "3"):
            # conditions: (theta)i = al and (theta)f = bh
            qi = (self.qi[0], self.qi[1], self.a[0])
            qf = (self.qf[0], self.qf[1], self.b[1])
            self.shortest_path(qi, qf, self.R)

        elif (parameter == "4"):
            # conditions: al < (theta)i < ah and (theta)f = bl
            qf = (self.qf[0], self.qf[1], self.b[0])

            interval_step = 0
            for t in np.arange(self.a[0], self.a[1], self.step):
                interval_step = interval_step + t
                qi = (self.qi[0], self.qi[1], self.a[0]+interval_step)
                self.shortest_path(qi, qf, self.R)

        elif (parameter == "5"):
            # conditions: al < (theta)i < ah and  bl < (theta)f < bh
            interval_step_a = 0
            interval_step_b = 0

            for ta in np.arange(self.a[0], self.a[1], self.step):
                interval_step_a = interval_step_a + ta
                qi = (self.qi[0], self.qi[1], self.a[0]+interval_step_a)
                for tb in np.arange(self.b[0], self.b[1], self.step):
                    interval_step_b = interval_step_b + tb
                    qf = (self.qf[0], self.qf[1], self.b[0]+interval_step_b)
                    self.shortest_path(qi, qf, self.R)

        elif (parameter == "6"):
            # conditions: al < (theta)i < ah and (theta)f = bh
            qf = (self.qf[0], self.qf[1], self.b[1])

            interval_step = 0
            for t in np.arange(self.a[0], self.a[1], self.step):
                interval_step = interval_step + t
                qi = (self.qi[0], self.qi[1], self.a[0]+interval_step)
                self.shortest_path(qi, qf, self.R)

        elif (parameter == "7"):
            # conditions: (theta)i = ah and (theta)f = bl
            qi = (self.qi[0], self.qi[1], self.a[1])
            qf = (self.qf[0], self.qf[1], self.b[0])
            self.shortest_path(qi, qf, self.R)

        elif (parameter == "8"):
            # conditions: (theta)i = ah and bl < (theta)f < bh
            qi = (self.qi[0], self.qi[1], self.a[1])

            interval_step = 0
            for t in np.arange(self.b[0], self.b[1], self.step):
                interval_step = interval_step + t
                qf = (self.qf[0], self.qf[1], self.b[0]+interval_step)
                self.shortest_path(qi, qf, self.R)

        elif (parameter == "9"):
            # conditions: (theta)i = ah and (theta)f = bh
            qi = (self.qi[0], self.qi[1], self.a[1])
            qf = (self.qf[0], self.qf[1], self.b[1])
            self.shortest_path(qi, qf, self.R)

        else:
            print("paramter is not one of the nine cases")

    # def update_path(self, dubins_path):
    #     # print(dubins_path.path())
    #     return

    #     # length_of_segments = dubins_path.params
    #     # type_of_path = dubins_path.type
    #     # if (sum(length_of_segments) < self.best_path_cost):
    #     #     self.best_path_cost = sum(length_of_segments)
    #     #     self.best_path_type = type_of_path
    #     #     self.params = length_of_segments
    #     #     return
    #     # else:
    #     #     return

    def shortest_path(self, qi, qf, radius):
        pt1 = pydubins.Waypoint(qi[0], qi[1], qi[2])
        pt2 = pydubins.Waypoint(qf[0], qf[1], qf[2])
        
        param = pydubins.calcDubinsPath(pt1, pt2, 1, 20)

        # update optimization guess based on the calculated dubins path
        cost = param.seg_final[0]+param.seg_final[1]+param.seg_final[2]
        if (cost < self.best_path_cost):
            self.best_param = param
            self.best_path_cost = param.seg_final[0] + param.seg_final[1] + param.seg_final[2]
        # path = copied_dubins.dubins_traj(param,1) # Used to plot the data

    # def optimize(self, param):
    #     cost = param.seg_final[0]+param.seg_final[1]+param.seg_final[2]
    #     if (cost < self.best_path_cost):
    #         self.best_param = param
    #         self.best_path_cost = param.seg_final[0] + param.seg_final[1] + param.seg_final[2]

    def results(self):
        print("Length of shortest path: ", self.best_path_cost)
        print("Type of segment: ", self.best_param.type)
        print("Length of segments: ", self.best_param.seg_final[0], ", ", self.best_param.seg_final[1], ", ", self.best_param.seg_final[2], ", ")

    def draw_path(self):
        path = pydubins.dubins_traj(self.best_param,0.1)

        # Plot the results
        plt.plot(self.qi[0],self.qi[1],'kx')
        plt.plot(self.qf[0],self.qf[1],'kx')
        plt.plot(path[:,0],path[:,1],'b-')

        self.plot_line_segment(self.qi, self.a[0])
        self.plot_line_segment(self.qi, self.b[1])
        self.plot_line_segment(self.qf, self.b[0])
        self.plot_line_segment(self.qf, self.b[1])

    

        plt.grid(True)
        plt.axis("equal")
        plt.title('Dubin\'s Curves Trajectory Generation')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()

    def plot_line_segment(self, q, angle):
        end_point = (q[0] + 0.5 * np.cos(angle), q[1] + 0.5 * np.sin(angle))
    
        # Plot the line segment
        plt.plot([q[0], end_point[0]], [q[1], end_point[1]], color='red')

def main():
    # minimum turning radius
    R = 1.0

    # initial position
    xi = 0.0
    yi = 0.0
    qi = (xi, yi)

    # final position
    xf = 5.0
    yf = 0.0
    qf = (xf, yf)

    # initial heading interval
    al = math.pi/6
    ah = math.pi/5
    a = (al, ah)

    # final heading interval
    bl = math.pi/2
    bh = math.pi
    b = (bl, bh)

    # solve dubins interval problem
    DubinsInterval(R, qi, qf, a, b)

if (__name__ == "__main__"):
    main()