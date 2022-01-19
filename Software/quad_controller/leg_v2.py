import math
import matplotlib.pyplot as plt

import logging
log = logging.getLogger(__name__)
from typing import NewType
from enum import Enum

import config


LegName = NewType('LegId', str)

class LegId (Enum):
    RF = 0
    LF = 1
    RR = 2
    LR = 3


class Leg:

    def __init__(self, id: LegId):
        self.id = id
        self.l3 = config.leg_upper_length

        self.move_enabled = False
        self.sym_enabled = False

        self.last_th1 = 0
        self.last_th2 = 0
        self.last_th3 = 0

    def move_to(self, foot_point):
        (th1, th2, th3) = kinematics.leg_inverse(foot_point)

        if self.move_enabled:
            servo.setangle(chan_th1, th1)
            servo.setangle(chan_th2, th2)
            servo.setangle(chan_th3, th3)

        if sym_enabled:
            sym.setangle(chan_th1, th1)
            sym.setangle(chan_th2, th2)
            sym.setangle(chan_th3, th3)

        self.last_th1 = th1
        self.last_th1 = th1
        self.last_th1 = th1

    def enable_moves(self, allowed: bool=True):
        self.move_enabled = allowed

    def enable_sym(self, allowed: bool=True):
        self.sym_enabled = allowed


class Legs:

    def __init__(self):

        self.leg_list = []

        for leg_id in LegId:
            self.leg_list.append(self.leg(leg_id))
        return

    def move_to(self, foot_point_lf, foot_point_rf,
                      foot_point_lr, foot_point_rr):

        self.leg_list(LegId.LF).move_to(foot_point_lf)
        self.leg_list(LegId.LF).move_to(foot_point_rf)
        self.leg_list(LegId.LR).move_to(foot_point_lr)
        self.leg_list(LegId.RR).move_to(foot_point_rr)
        return

    def enable_moves(self, allowed: bool = True):
        for lg in self.leg_list:
            lg.enable_moves(allowed)
        return

    def enable_sys(self, allowed: bool = True):
        for lg in self.leg_list:
            lg.enable_sym(allowed)
        return
