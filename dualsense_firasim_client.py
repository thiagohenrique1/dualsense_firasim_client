import socket
from pydualsense import *
import struct
import math
from math import remainder
import sys
sys.path.insert(0, './msg')
from packet_pb2 import Environment, Packet
from command_pb2 import Commands, Command

RECEIVE_UDP_IP = "224.0.0.1"
SEND_UDP_IP = "127.0.0.1"
UDP_RECEIVE_PORT = 10002
UDP_SEND_PORT = 20011

sock_receive = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock_receive.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_receive.bind((RECEIVE_UDP_IP, UDP_RECEIVE_PORT))

sock_send = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

env = Environment()

class Point(object):
    def __init__(self, x = 0, y = 0):
        self.x = x
        self.y = y

dualsense = pydualsense()
dualsense.init()

def analog_to_vector(x, y):
    x = x/128.0 * 1.5
    y = y/128.0 * 1.5
    size = math.sqrt(pow(x, 2) + pow(y, 2))
    theta = math.atan2(y, x)
    return size, theta

def vector_control(size, target_theta, robot_theta):
    lin_vel = size
    error = target_theta - robot_theta
    error_wrap = math.remainder(error, 2 * math.pi)
    if (abs(error_wrap) > math.pi / 2):
        error_wrap = math.remainder(target_theta - (robot_theta + math.pi), 2 * math.pi)
        lin_vel = -size
    return lin_vel * math.cos(error_wrap) * 0.5, error_wrap * 10 # lin vel, ang vel

ROBOT_LENGTH = 0.08
def to_wheel_velocity(lin_vel, ang_vel):
    Vl = lin_vel - (ang_vel * ROBOT_LENGTH)/2
    Vr = lin_vel + (ang_vel * ROBOT_LENGTH)/2
    return Vl, Vr

WHEEL_RADIUS = 0.02
def to_angular_wheel_velocity(Vl, Vr):
    Wl = Vl / WHEEL_RADIUS
    Wr = Vr / WHEEL_RADIUS
    return Wl, Wr

def dualsense_control(robot_orientation, dualsense_x, dualsense_y):
    if (abs(dualsense_x) > 10 or abs(dualsense_y) > 10):
        size, theta = analog_to_vector(dualsense_x, dualsense_y)
        lin_vel, ang_vel = vector_control(size, theta, robot_orientation)
        Vl, Vr = to_wheel_velocity(lin_vel, ang_vel)
        Wl, Wr = to_angular_wheel_velocity(Vl, Vr)
        return Wl, Wr
    else:
        return 0, 0

def dualsense_to_cmd(frame, robot_id, yellowteam, dualsense_x, dualsense_y, spin_kick):
    cmd = Command()
    cmd.id = robot_id
    cmd.yellowteam = yellowteam

    robot = frame.robots_yellow[robot_id] if yellowteam else frame.robots_blue[robot_id]
    target = Point(-0.75, 0) if yellowteam else Point(0.75, 0)

    if (spin_kick):
        cmd.wheel_left, cmd.wheel_right = spin_kick_to_target(robot, frame.ball, target)
    else:
        cmd.wheel_left, cmd.wheel_right = dualsense_control(robot.orientation, dualsense_x, dualsense_y)
    return cmd

def send_stop(packet, robot_id, yellowteam):
    cmd = Command()
    cmd.id = robot_id
    cmd.yellowteam = yellowteam
    cmd.wheel_left, cmd.wheel_right = 0, 0
    packet.cmd.robot_commands.append(cmd)

def spin_kick_to_target(position, ball, target):
    robot_to_ball = math.atan2(ball.y - position.y, ball.x - position.x)
    ball_to_target = math.atan2(target.y - ball.y, target.x - ball.x)
    angle = math.remainder(ball_to_target - robot_to_ball, 2 * math.pi)
    if (angle > 0):
        return -50, 50
    else:
        return 50, -50

while True:
    data, addr = sock_receive.recvfrom(2048)
    env.ParseFromString(data)
    frame = env.frame
    ball = frame.ball
    
    packet = Packet()
    yellowteam = False
    left_id = 0
    right_id = 1
    third_robot_id = 2
    if (dualsense.state.L1):
        send_stop(packet, left_id, yellowteam)
        left_id = third_robot_id
    elif (dualsense.state.R1):
        send_stop(packet, right_id, yellowteam)
        right_id = third_robot_id
    else:
        send_stop(packet, third_robot_id, yellowteam)
    cmd = dualsense_to_cmd(frame, left_id, yellowteam, dualsense.state.LX, -dualsense.state.LY, dualsense.state.L2)
    packet.cmd.robot_commands.append(cmd)
    cmd = dualsense_to_cmd(frame, right_id, yellowteam, dualsense.state.RX, -dualsense.state.RY, dualsense.state.R2)
    packet.cmd.robot_commands.append(cmd)
        
    packet_bytes = packet.SerializeToString()
    sock_send.sendto(packet_bytes, (SEND_UDP_IP, UDP_SEND_PORT))
    
