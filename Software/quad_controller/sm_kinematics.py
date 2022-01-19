"""
Compute forward and inverse kinematics for a spot micro robot.

Note that the SpotMicro has "special" kinematics, and unlike many similar
quadruped robots not one but two displacements between the first to joints so
the rotational axis of the first two joints do not intersect.  (They mss by
abut 30 mm depending on the exact 3D files used to print the robot)

Credit:  This file was copied from a SpotMico simulation written by Florian Wilk.
         Only minor modifications were made to adapt it to the current environment
         and to add comments, docstrings and tests.
         Many of the comments were from a document written by Florian Wilk.

Change Log, relative to original work by Wilk.
    1)  Replace print statements with calling to logging.
"""

from mpl_toolkits import mplot3d
import numpy as np
from math import *
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

import logging

import config

log = logging.getLogger(__name__)

fig = plt.figure()
data = np.random.rand(2, 25)

def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

class Kinematic:

    def __init__(self):

        self.l1 = 1000.0 * config.j1_to_j2_horizontal_offset
        self.l2 = 1000.0 * config.j1_to_j2_vertical_offset
        self.l3 = 1000.0 * config.leg_upper_length
        self.l4 = 1000.0 * config.leg_lower_length

        # FIXME -- Constants need to come from config
        self.L = 260
        self.W = 78

    def bodyIK(self, omega: float, phi: float, psi: float,
                     xm: float, ym: float, zm: float):
        Rx = np.array([[1,0,0,0],
                    [0,np.cos(omega),-np.sin(omega),0],
                    [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                    [0,1,0,0],
                    [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                    [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz = Rx.dot(Ry.dot(Rz))

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        sHp=np.sin(pi/2)
        cHp=np.cos(pi/2)
        (L,W)=(self.L,self.W)

        return([Tm.dot(np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]]))])

    def legIK(self, point: (float, float, float)):
        (x, y, z) = (point[0], point[1], point[2])
        (l1, l2, l3, l4) = (self.l1, self.l2, self.l3, self.l4)
        try:
            F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
        except ValueError:
            log.warning("Error in legIK with x {} y {} and l1 {}".format(x, y, l1))
            F = l1

        G = F - l2
        H = sqrt(G ** 2 + z ** 2)
        theta1 = -atan2(y, x) - atan2(F, -l1)

        D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
        try:
            theta3 = acos(D)
        except ValueError:
            log.warnng("Error in legIK with x {} y {} and D {}".format(x, y, D))
            theta3 = 0

        theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))

        return (theta1, theta2, theta3)

    def calcLegPoints(self, angles: (float, float, float)):
        (l1, l2, l3, l4) = (self.l1, self.l2, self.l3, self.l4)
        (theta1, theta2, theta3) = angles
        theta23 = theta2 + theta3

        T0 = np.array([0, 0, 0, 1])
        T1 = T0 + np.array([-l1 * cos(theta1), l1 * sin(theta1), 0, 0])
        T2 = T1 + np.array([-l2 * sin(theta1), -l2 * cos(theta1), 0, 0])
        T3 = T2 + np.array([-l3 * sin(theta1) * cos(theta2), -l3 * cos(theta1) * cos(theta2), l3 * sin(theta2), 0])
        T4 = T3 + np.array([-l4 * sin(theta1) * cos(theta23), -l4 * cos(theta1) * cos(theta23), l4 * sin(theta23), 0])

        return np.array([T0, T1, T2, T3, T4])

    def drawLegPoints(self, p):
        plt.plot([x[0] for x in p], [x[2] for x in p], [x[1] for x in p], 'k-', lw=3)
        plt.plot([p[0][0]], [p[0][2]], [p[0][1]], 'bo', lw=2)
        plt.plot([p[4][0]], [p[4][2]], [p[4][1]], 'ro', lw=2)

    def drawLegPair(self, Tl, Tr, Ll, Lr):
        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.drawLegPoints([Tl.dot(x) for x in self.calcLegPoints(self.legIK(np.linalg.inv(Tl).dot(Ll)))])
        self.drawLegPoints(
            [Tr.dot(Ix.dot(x)) for x in self.calcLegPoints(self.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))])

    def drawRobot(self, Lp, angles: (float, float, float), center: (float, float, float)):
        (omega, phi, psi) = angles
        (xm, ym, zm) = center

        FP = [0, 0, 0, 1]
        (Tlf, Trf, Tlb, Trb) = self.bodyIK(omega, phi, psi, xm, ym, zm)
        CP = [x.dot(FP) for x in [Tlf, Trf, Tlb, Trb]]

        CPs = [CP[x] for x in [0, 1, 3, 2, 0]]
        plt.plot([x[0] for x in CPs], [x[2] for x in CPs], [x[1] for x in CPs], 'bo-', lw=2)

        self.drawLegPair(Tlf, Trf, Lp[0], Lp[1])
        self.drawLegPair(Tlb, Trb, Lp[2], Lp[3])

    def calcIK(self, Lp, angles, center):
        (omega, phi, psi) = angles
        (xm, ym, zm) = center

        (Tlf, Trf, Tlb, Trb) = self.bodyIK(omega, phi, psi, xm, ym, zm)

        Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        return np.array([self.legIK(np.linalg.inv(Tlf).dot(Lp[0])),
                         self.legIK(Ix.dot(np.linalg.inv(Trf).dot(Lp[1]))),
                         self.legIK(np.linalg.inv(Tlb).dot(Lp[2])),
                         self.legIK(Ix.dot(np.linalg.inv(Trb).dot(Lp[3])))])


if __name__ == "__main__":
    setupView(500).view_init(elev=12., azim=28)

    vertical = 200
    stance_width = 150
    stance_length= 260

    stance_halfwidth  = stance_width  / 2.0
    stance_halflength = stance_length / 2.0

    Lp = np.array([[ stance_halflength, -vertical,  stance_halfwidth, 1],
                   [ stance_halflength, -vertical, -stance_halfwidth, 1],
                   [-stance_halflength, -vertical,  stance_halfwidth, 1],
                   [-stance_halflength, -vertical, -stance_halfwidth, 1]])
    Kinematic().drawRobot(Lp, (0, 0, 0), (0, 0, 0))
    plt.show()
