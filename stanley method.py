"""
기하학적 경로추종 방법 Stanley Method
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# parameter
time_tic = 0.1  # time
k = 2.0  # control gain
velo = 10

# input
end_x = 50
end_y = 50

start_x = 0.0
start_y = 0.0

front_x = 0.0
front_y = 0.0

yaw = math.radians(0)

theta = math.radians(45)

max_steering = math.radians(30)

list_front_x = []
list_front_y = []
list_yaw = []
list_t = []

linear_d = math.hypot(end_x - start_x, end_y - start_y)  # Linear distance between start and end point
R = linear_d / (2 * math.sin(0.5 * theta))


def calc_circle_point(point):  # 시작점과 도착점 사이 각도 theta 원의 좌표
    a = end_x ** 2 + end_y ** 2
    b = (-2) * end_x * (R ** 2 + 0.25 * (linear_d ** 2) - (linear_d / (2 * math.tan(0.5 * theta))) ** 2)
    c = (R ** 2 + 0.25 * (linear_d ** 2) - (linear_d / (2 * math.tan(0.5 * theta))) ** 2) ** 2 - (R ** 2) * (
                end_y ** 2)

    if end_x * end_y > 0:
        circle_x = ((-b) + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        circle_y = -math.sqrt(R ** 2 - circle_x ** 2)

    else:
        circle_x = ((-b) + math.sqrt(b ** 2 - 4 * a * c)) / (2 * a)
        circle_y = math.sqrt(R ** 2 - circle_x ** 2)

    if point == "x":  # x 좌표를 return
        return circle_x

    elif point == "y":  # y 좌표를 return
        return circle_y


def cross_track_error():  # 앞바퀴와 경로의 횡방향 오차

    e = math.hypot(front_x - calc_circle_point("x"), front_y - calc_circle_point("y")) - R

    return e


def steering():
    if end_x > 0 and end_y > 0:  # end point 제 1사분면
        curvature = math.acos((calc_circle_point("x") - front_x) / (cross_track_error() + R))

    elif end_x < 0 < end_y:  # end point 제 2사분면
        curvature = math.acos(
            (calc_circle_point("x") - front_x) / (cross_track_error() + R)) * (-1)

    elif end_x < 0 and end_y < 0:  # end point 제 3사분면
        curvature = math.acos(
            (calc_circle_point("x") - front_x) / (cross_track_error() + R)) - math.pi

    else:  # end point 제 4사분면
        curvature = math.pi - math.acos(
            (calc_circle_point("x") - front_x) / (cross_track_error() + R))

    psi = curvature - yaw
    delta = psi + math.atan2(k * cross_track_error() / velo, 1.0)

    limit_steering(delta)

    return delta


def limit_steering(delta):

    if delta % math.pi * 2 > math.pi:
        delta = delta % math.pi - math.pi

    elif delta % math.pi * 2 < -math.pi:
        delta = delta % math.pi + math.pi

    if delta > max_steering:
        delta = max_steering

    elif delta < -max_steering:
        delta = - max_steering

    return delta


def plot_arrow(x, y, alpha, length=1.0, width=0.5, fc="r", ec="k"):

    plt.arrow(x, y, length * math.sin(alpha), length * math.cos(alpha),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)


# output

for time in np.arange(0.0, 100, time_tic):

    yaw += steering()  # heading update
    front_x += velo * time_tic * math.sin(yaw)  # x 위치 좌표 update
    front_y += velo * time_tic * math.cos(yaw)  # y 위치 좌표 update

    plt.cla()
    plt.grid(True)
    plt.title('stanley method')
    plt.xlim(0, end_x)
    plt.ylim(0, end_x)
    circle = patches.Circle((calc_circle_point("x"), calc_circle_point("y")), R, fill=False)
    plt.gca().add_patch(circle)
    plot_arrow(front_x, front_y, yaw)
    plt.pause(0.001)

    if (end_x - front_x) / end_x < 0.01 and (end_y - front_y) / end_y < 0.01:  # 목적지 도달 break

        break

plt.show()
