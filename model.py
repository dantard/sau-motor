#!/bin/env python

import time
from random import randint, random

import numpy as np
import rclpy
from control import tf, forced_response
from control.matlab import lsim
from matplotlib import pyplot as plt

from basic_node import BasicNode

period = 0.01

class Model(BasicNode):
    def __init__(self, model):
        super().__init__('Model', 'error', 'feedback')
        self.model = model
        self.X = [0, 0]
        self.y = []
        self.t = []
        self.started = False
        self.elapsed = 0

    def receive(self, error):
        self.started = True
        error = error.data
        y, t, self.X = lsim(self.model, U=error, T=[0, period], X0=self.X[1])
        time.sleep(random() / 100)
        self.send(y[1])

        self.y.append(y[1])
        self.elapsed += period
        #self.t.append(self.elapsed)
        self.t.append(time.time())


    def get_y(self):
        return self.y

    def get_t(self):
        return self.t

    def get_started(self):
        return self.started


def main(args=None):
    rclpy.init(args=args)

    s = tf('s')
    L, R, J, B, Ke, Ki = 0.1, 0.01, 0.1, 0.1, 0.1, 0.1
    G = 1 / (L * s + R) * Ki * 1 / (J * s + B)

    a = 10
    b = 50
    G = 1 / (s + a) / (s + b) /s * 4000

    model = Model(G)

    elapsed = 0
    while elapsed < 2 * 4 / (a+b)/2*4*4:
        rclpy.spin_once(model, timeout_sec=0.01)
        if model.get_started():
            elapsed += 0.01

    y, t = model.get_y(), model.get_t()
    t1 = [t2 - t[0] for t2 in t]
    plt.plot(t1, y)

    # Time Generation
    t = np.arange(0, t1[-1], period)

    # One shot simulation
    yt, tt, xt = lsim(G / (1 + G), U=100.0, T=t)
    plt.plot(tt, yt)

    # Closed loop simulation

    error = 0
    X = [0, 0]
    closed_loop_sim_y = []
    for i in range(len(t)):
        y, t11, X = lsim(G, U=error, T=[0, period], X0=X[1])
        error = 100.0 - y[1]
        closed_loop_sim_y.append(y[1])

    plt.plot(t, closed_loop_sim_y)

    plt.show()
    model.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
