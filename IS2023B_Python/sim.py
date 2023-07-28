import numpy as np
from math import sqrt, exp, log
import matplotlib.pyplot as plt
from matplotlib.backend_bases import LocationEvent
from matplotlib.patches import Rectangle
from warnings import warn

LENGTH = 500
WIDTH = 500
HALF_SQUARE = LENGTH / 2 + 50
UNIT = 50
V_VOICE = 340
CLK_FREQ = 128000000
Q1 = np.array([HALF_SQUARE, HALF_SQUARE])
Q2 = np.array([-HALF_SQUARE, HALF_SQUARE])
Q3 = np.array([-HALF_SQUARE, -HALF_SQUARE])
Q4 = np.array([HALF_SQUARE, -HALF_SQUARE])
MICROPHONES = np.array([Q1, Q2, Q3, Q4])
POINT_DIST = np.zeros(4)
CORRECT_POINT_DIST = np.zeros(4)
O = np.array([0, 0])
G_VECTOR = np.zeros((4, 2))
RW, RH = 8., 8.
P = None
total_steps = 50
steps = 15
P_skip_num = 0

fig, ax = plt.subplots()

def gradient_descent(pi:np.ndarray, step:int):
    sum = 0
    pi1 = pi
    gradients_list = np.zeros((4, 2))
    for name, p in enumerate(MICROPHONES):
        POINT_DIST[name] = dist(p, pi) - dist(MICROPHONES[P_skip_num], pi) - (CORRECT_POINT_DIST[name] - CORRECT_POINT_DIST[P_skip_num])
        sum += abs(POINT_DIST[name])
    for name, p in enumerate(MICROPHONES):
        POINT_DIST[name] /= sum
        gradients_list[name] = G_VECTOR[name] * POINT_DIST[name] * (steps + sum / 20) *  (1 - log(1 + exp(step / 5 - 10) ** 3) / 0.5)#  (exp(-step / 8) + 0.5)
        pi1 += gradients_list[name]
    return pi1

def gradient_descent_wrapper():
    global P_skip_num, G_VECTOR
    P_skip_num = np.argmin(CORRECT_POINT_DIST)
    P0 = MICROPHONES[P_skip_num] * 5 / 12
    Pi = P0
    for i in range(4):
        G_VECTOR[i] = MICROPHONES[i] - MICROPHONES[P_skip_num]
        if i != P_skip_num:
            G_VECTOR[i] /= dist(O, G_VECTOR[i]) * sqrt(3)
    for step in range(total_steps):
        Pi = gradient_descent(Pi, step)
        if step > total_steps - 5:
            print(f"{step}th dist:{dist(Pi, P)}")
        ax.scatter(Pi[0], Pi[1], s=1, c="y")
        plt.draw()
    print(f"final:{Pi}")

def draw_target():
    ax.set_aspect(1.)
    ax.scatter(Q1[0], Q1[1], 30, "r", "o")
    ax.scatter(Q2[0], Q2[1], 30, "r", "o")
    ax.scatter(Q3[0], Q3[1], 30, "r", "o")
    ax.scatter(Q4[0], Q4[1], 30, "r", "o")
    for i in range(LENGTH // UNIT + 1):
        ax.hlines(i * UNIT - LENGTH / 2, -LENGTH / 2, LENGTH / 2, "black", "dashdot")
        ax.vlines(i * UNIT - WIDTH / 2, -WIDTH / 2, WIDTH / 2, "black", "dashdot")

def dist(p1:np.ndarray, p2:np.ndarray):
    return np.sqrt(np.sum(np.power(p2 - p1, 2)))

def onclick(event:LocationEvent):
    global P
    if event.inaxes:
        P = np.array([event.xdata, event.ydata])
        print(f"origin:{P}")
        artists_object = Rectangle(xy=[P[0], P[1] - RH / sqrt(2)], width=RW, height=RH, angle=45, facecolor="g")
        ax.add_artist(artists_object)
        plt.draw()
        for index, p in enumerate(MICROPHONES):
            CORRECT_POINT_DIST[index] = dist(p, P)
            print(f"to {index}:{CORRECT_POINT_DIST[index]}")
        if abs(P[0]) > LENGTH / 2 or abs(P[1]) > WIDTH / 2:
            warn("warning: out of bound!")
        else:
            gradient_descent_wrapper()
        print()
    else:
        ax.cla()
        draw_target()
        print("clear")
        plt.draw()

cid = fig.canvas.mpl_connect('button_press_event', onclick)
draw_target()
plt.show()