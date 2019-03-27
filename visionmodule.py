# -*- coding: utf-8 -*-
"""
@Brief: This is a vision module(single robot) for RoboCup Small Size League 
@Version: grSim 4 camera version
@author: Wang Yunkai
"""

import socket
from time import sleep
import messages_robocup_ssl_wrapper_pb2 as wrapper
import numpy as np

MULTI_GROUP = '224.5.23.2'
VISION_PORT = 23333 # Athena vision port
ROBOT_ID = 6

class VisionModule:
    def __init__(self, VISION_PORT=23333, SENDERIP = '0.0.0.0'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.sock.bind((SENDERIP,VISION_PORT))
        self.sock.setsockopt(socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            socket.inet_aton(MULTI_GROUP) + socket.inet_aton(SENDERIP))
        self.robot_info = [0, 0, 0]
        self.ball_info = [0, 0]

    def receive(self):
        data, addr = self.sock.recvfrom(65535)
        sleep(0.001) # wait for reading
        return data

    def get_obstacles(self,num_list=np.array([])):
        data = self.receive()
        package = wrapper.SSL_WrapperPacket()
        package.ParseFromString(data)
        obstacles = np.array([[],[]])

        detection = package.detection
        robots = detection.robots_blue  # repeat
        for robot in robots:

            for num in num_list:
                    if robot.robot_id == num:
                        obstacles[0] = np.append(obstacles[0],robot.x / 1000.0)
                        obstacles[1] = np.append(obstacles[1],robot.y / 1000.0)
                        obstacles[2] = np.append(obstacles[2],robot.orientation / 1000.0)
        return obstacles[0:2]
    def get_info(self, ROBOT_ID):
        data = self.receive()
        
        package = wrapper.SSL_WrapperPacket()
        package.ParseFromString(data)

        detection = package.detection
        robots = detection.robots_blue  # repeat
        for robot in robots:
            if robot.robot_id == ROBOT_ID:
                self.robot_info[0] = robot.x/1000.0
                self.robot_info[1] = robot.y/1000.0
                self.robot_info[2] = robot.orientation
                # self.robot_info[3] = robot.vel_x/1000.0
                # self.robot_info[4] = robot.vel_y/1000.0
                # self.robot_info[5] = robot.rotate_vel
                print('Robot', robot.confidence)
        
        balls = detection.balls # not repeat
        for ball in balls:
            self.ball_info[0] = ball.x/1000.0
            self.ball_info[1] = ball.y/1000.0
            print(self.ball_info)
        return np.array([self.robot_info[0],self.robot_info[1]])
        # self.ball_info[2] = ball.vel_x/1000.0
        # self.ball_info[3] = ball.vel_y/1000.0
        # print('Ball', ball.confidence)

  
if __name__ == '__main__':


    vision = VisionModule(VISION_PORT)

    while True:
        vision.get_info(ROBOT_ID)
        # print(vision.robot_info)
        # # print(vision.ball_info)
