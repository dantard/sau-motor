#!/bin/env python

import threading
import time
from random import randint

import pygame
import math

import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from scipy import rand
from std_msgs.msg import Int64MultiArray, Int64

# Initialize Pygame
pygame.init()
from rclpy.node import Node

# Constants
WIDTH, HEIGHT = 800, 800
ROBOT_WIDTH = 40
ROBOT_HEIGHT = 30
WHEEL_RADIUS = 5
MAX_LINEAR_SPEED = 0.25  # Max linear speed in pixels per frame
MAX_ROTATIONAL_SPEED = 0.25  # Max rotational speed in radians per frame

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Screen setup
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Differential Drive Robot Simulation")

pound = True


# Robot class
class Robot(Node):
    def __init__(self, x, y, theta):
        super().__init__('robot')

        profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(nanoseconds=10000 * 1e9),
            lifespan=Duration(nanoseconds=10000 * 1e9),
            depth=10
        )

        self.init_x, self.init_y, self.init_theta = x, y, theta
        self.x = x
        self.y = y
        self.theta = theta
        self.linear_speed = 0
        self.rotational_speed = 0
        self.poses = []
        self.commands = []
        self.subscription_hf = self.create_subscription(
            Int64MultiArray, "/outta1" if pound else "/out1", self.hf_callback, 10 if pound else profile)

        self.state = 0
        self.cnt = 0
        self.record = 0

    lost = 0
    prev = 0

    def hf_callback(self, msg):
        a1, a2, self.cnt = msg.data[1], msg.data[2], msg.data[3]
        lost = self.lost + self.cnt - self.prev - 1
        self.lost != lost and print("Lost: ", lost)
        self.prev = self.cnt
        self.lost = lost

        if a1 == -1 and a2 == -1:
            if self.record == 1:
                self.record = 2
            elif self.record == 3:
                self.record = 0
            self.linear_speed = 0
            self.rotational_speed = 0
            self.theta = self.init_theta
            self.x = self.init_x
            self.y = self.init_y

        else:
            if self.record == 2:
                self.record = 3

            self.linear_speed = a1 / 1000000 * 4
            self.rotational_speed = a2 / 1000000 * 4

        if self.record == 3:
            self.poses.append((self.x, self.y))

    def update(self):
        self.x += self.linear_speed * math.cos(self.theta)
        self.y += self.linear_speed * math.sin(self.theta)
        self.theta += self.rotational_speed / 240

    def set_record(self, value):
        self.record = value

    def apply_figure_8_velocity(self, t, T):
        # T is the period for one loop of the figure-8
        omega = 2 * math.pi / T  # Angular frequency
        self.linear_speed = MAX_LINEAR_SPEED
        self.rotational_speed = MAX_ROTATIONAL_SPEED * math.sin(omega * t)

    def draw(self, screen):
        # Draw the robot as a rectangle
        points = [
            (self.x + ROBOT_WIDTH / 2 * math.cos(self.theta) - ROBOT_HEIGHT / 2 * math.sin(self.theta),
             self.y + ROBOT_WIDTH / 2 * math.sin(self.theta) + ROBOT_HEIGHT / 2 * math.cos(self.theta)),
            (self.x - ROBOT_WIDTH / 2 * math.cos(self.theta) - ROBOT_HEIGHT / 2 * math.sin(self.theta),
             self.y - ROBOT_WIDTH / 2 * math.sin(self.theta) + ROBOT_HEIGHT / 2 * math.cos(self.theta)),
            (self.x - ROBOT_WIDTH / 2 * math.cos(self.theta) + ROBOT_HEIGHT / 2 * math.sin(self.theta),
             self.y - ROBOT_WIDTH / 2 * math.sin(self.theta) - ROBOT_HEIGHT / 2 * math.cos(self.theta)),
            (self.x + ROBOT_WIDTH / 2 * math.cos(self.theta) + ROBOT_HEIGHT / 2 * math.sin(self.theta),
             self.y + ROBOT_WIDTH / 2 * math.sin(self.theta) - ROBOT_HEIGHT / 2 * math.cos(self.theta))
        ]
        pygame.draw.polygon(screen, RED, points)
        pygame.draw.lines(screen, BLACK, True, points, 2)

        font1 = pygame.font.SysFont('chalkduster.ttf', 24)
        img1 = font1.render(str(self.cnt), True, BLACK)

        if len(self.poses) > 2:
            pygame.draw.lines(screen, BLACK, False, self.poses, 3)

        screen.blit(img1, (30, 30))

        # Draw wheels
        left_wheel_x = self.x - ROBOT_WIDTH / 2 * math.cos(self.theta) + WHEEL_RADIUS * math.sin(self.theta)
        left_wheel_y = self.y - ROBOT_WIDTH / 2 * math.sin(self.theta) - WHEEL_RADIUS * math.cos(self.theta)
        right_wheel_x = self.x + ROBOT_WIDTH / 2 * math.cos(self.theta) + WHEEL_RADIUS * math.sin(self.theta)
        right_wheel_y = self.y + ROBOT_WIDTH / 2 * math.sin(self.theta) - WHEEL_RADIUS * math.cos(self.theta)

        # pygame.draw.circle(screen, BLACK, (int(left_wheel_x), int(left_wheel_y) + 5), WHEEL_RADIUS)
        # pygame.draw.circle(screen, BLACK, (int(right_wheel_x), int(right_wheel_y) + 5), WHEEL_RADIUS)


# Main loop
def main():
    rclpy.init()
    commands = yaml.load(open("commands.yaml", "r"), Loader=yaml.FullLoader)
    pos = yaml.load(open("poses.yaml", "r"), Loader=yaml.FullLoader)

    posi = yaml.load(open("poses-one-hop.yaml", "r"), Loader=yaml.FullLoader)

    clock = pygame.time.Clock()
    running = True
    pygame.key.set_repeat(1000, 1000)
    # Create a robot instance
    robot = Robot(140, 20, 0)
    start_time = time.time()
    last_time = 0
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get key states
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            robot.linear_speed = min(MAX_LINEAR_SPEED, robot.linear_speed + 1)
        elif keys[pygame.K_DOWN]:
            robot.linear_speed = max(-MAX_LINEAR_SPEED, robot.linear_speed - 1)

        if keys[pygame.K_LEFT]:
            robot.rotational_speed = max(-MAX_ROTATIONAL_SPEED, robot.rotational_speed - 1)
        elif keys[pygame.K_RIGHT]:
            robot.rotational_speed = min(MAX_ROTATIONAL_SPEED, robot.rotational_speed + 1)

        if keys[pygame.K_r]:
            robot.set_record(1)

        if keys[pygame.K_s]:
            yaml.dump(robot.poses, open("poses-one-hop.yaml", "w"))

        # Update robot position
        robot.update()

        # Clear screen
        screen.fill(WHITE)

        # Draw robot
        robot.draw(screen)
        pygame.draw.lines(screen, GREEN, False, posi, 2)
        # Update display
        pygame.display.flip()

        # if robot.x > WIDTH or robot.x < 0 or robot.y > HEIGHT or robot.y < 0:
        #    running = False

        if False and (time.time_ns() - last_time) / 1e6 > 20:  # + randint(0, 10):
            last_time = time.time_ns()
            if len(commands) > 0:
                robot.linear_speed, robot.rotational_speed = commands.pop(0)
            # robot.apply_figure_8_velocity((time.time() - start_time) * 100, 2500 + (time.time() - start_time) * 100)

            # pos.append((robot.x, robot.y))
            # commands.append((robot.linear_speed, robot.rotational_speed))

        # robot.apply_figure_8_velocity((time.time() - start_time) * 800, 2500 + (time.time() - start_time) * 100)
        # Cap the frame rate
        rclpy.spin_once(robot, timeout_sec=0)
        clock.tick(200)
        # print("freq: ", clock.get_fps())

        # yaml.dump(commands, open("commands.yaml", "w"))
        # yaml.dump(pos, open("poses.yaml", "w"))
        # print(robot.x, robot.y, robot.theta)
    pygame.quit()


if __name__ == "__main__":
    main()
