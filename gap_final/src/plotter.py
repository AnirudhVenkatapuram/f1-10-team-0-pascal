#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input, AngleDistList, AngleDist

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(figsize = (8, 8), subplot_kw = {'projection':'polar'})
sc = ax.scatter([], [], s=5)
ax.set_title("LIve updating lidar")
ax.grid(True)
ax.set_rlim(0, 4.5)

data = [(0, 0, "yellow") for i in range(2000)]
arrows_data = [(0, 0, "black") for i in range(2000)]
arrows = []

def draw_arrows(angles, dist=2):
    global arrows, arrows_data

    for angle, dist, color in arrows_data:
        if dist > 0: 
            arrow = ax.annotate('', xy=(angle, dist), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
            arrows.append(arrow)

def callback_arrow(msg):
    global arrows_data
    arrows_data[msg.idx] = (msg.angle, msg.dist, msg.color)

def callback(msg):
    global data
    data[msg.idx] = (msg.angle, msg.dist, msg.color)

def update(frame):
    global data, arrows, arrows_data
    
    if (rospy.is_shutdown()):
        plt.close("all")
        return

    angles_rad, dists, colors = zip(*data)
    #angles_rad = np.deg2rad(angles)

    sc.set_offsets(np.c_[angles_rad, dists])
    sc.set_color(colors)

    for arrow in arrows:
        arrow.remove()

    arrows = []
    draw_arrows([0, math.pi/2])

    if frame % 3 == 0:
        data = [(0, 0, "yellow") for i in range(2000)]
        arrows_data = [(0, 0, "black") for i in range(2000)]

    print("UPDATE: " + str(frame))
    return sc,

rospy.init_node('plotter')
rospy.Subscriber('angle_dist', AngleDist, callback)
rospy.Subscriber('arrow', AngleDist, callback_arrow)

ani = FuncAnimation(fig, update, frames=np.arange(0, 100000), interval=5, blit=False)
plt.show()

rospy.spin()

