#!/bin/env python
import time
from random import randint, random

import rclpy

from basic_node import BasicNode


class Sum(BasicNode):
    def __init__(self):
        super().__init__('Sum', 'feedback', 'error')
        self.Vr = 1.0
        self.omega_t_minus_1 = 0
        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        error = self.Vr - self.omega_t_minus_1
        #if randint(0, 10) != 0:
        time.sleep(random() / 100)
        self.send(error)

    def control(self, vr):
        self.Vr = vr
        self.send(vr)

    def receive(self, value):
        self.omega_t_minus_1 = value.data
        #error = self.Vr - self.omega_t_minus_1
        #time.sleep(0.01)
        #self.send(error)



def main(args=None):
    rclpy.init(args=args)
    sum = Sum()
    sum.control(100.0)
    while rclpy.ok():
        rclpy.spin_once(sum, timeout_sec=0.0001)

    sum.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
