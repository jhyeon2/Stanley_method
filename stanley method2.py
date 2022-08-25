import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate

time = 0.0
time_tic = 0.1
k = 1.0     # control gain
velo = 5.0  # velocity [m/s]
L = 1.0

start_x = 0.0
start_y = 0.0

end_x = 50.0
end_y = 0.0

front_x = 0.0
front_y = 0.0

yaw = np.radians(0)
max_steering = np.radians(30)
max_velo = velo
min_velo = velo / 2

num = 10    # num 개의 랜덤한 좌표를 생성
n = 1000    # x 좌표를 n개로 나눔
dev = 5     # 경로 생성 표준편차


def make_road():   # 경로 생성

    target_x = np.linspace(start_x, end_x, num, endpoint=True)
    target_y = np.random.normal(start_y, dev, num-2)

    target_y = np.insert(target_y, 0, [start_y])
    target_y = np.append(target_y, [end_y])

    f_linear = interpolate.interp1d(target_x, target_y, kind='cubic')
    x_new = np.linspace(start_x, end_x, n, endpoint=True)
    y_new = f_linear(x_new)

    return x_new, y_new


def ctr_point():   # 횡방향 오차의 x y 좌표

    point_list = np.array([0.0] * n)

    for i in range(0, n):
        point_list[i] = np.hypot(road_x[i] - front_x, road_y[i] - front_y)

    point = np.argmin(point_list)

    return point


def cross_track_error():    # 경로와 현위치 사이의 횡방향 오차

    ctr = np.hypot(road_x[ctr_point()] - front_x, road_y[ctr_point()] - front_y)

    return ctr


def steering():     # 차량이 경로로 가기 위한 조향각 delta

    curvature = np.arctan2(
        (road_y[ctr_point()-1] - road_y[ctr_point()+1]) / (road_x[ctr_point()+1] - road_x[ctr_point()-1]), 1.0)

    if front_x == 0:
        curvature = np.arctan2(road_y[ctr_point()+1]/road_x[ctr_point()+1], 1.0) * (-1)

    elif front_x >= road_x[n-1]:
        curvature = np.arctan2(
            (road_y[ctr_point()-1] - road_y[ctr_point()]) / (road_x[ctr_point()] - road_x[ctr_point()-1]), 1.0)

    psi = curvature - yaw

    if curvature >= 0:
        delta = psi + np.arctan2(k * cross_track_error() / velo, 1.0)
    else:
        delta = psi - np.arctan2(k * cross_track_error() / velo, 1.0)

    delta = normalization_steering(delta)

    return delta


def normalization_steering(steering_angle):      # 조향각 정상화

    if steering_angle > max_steering:
        steering_angle = max_steering

    elif steering_angle < -max_steering:
        steering_angle = -max_steering

    return steering_angle


def update_velo():      # 속도 제어

    if abs(steering()) >= np.radians(5):
        velo_new = max(velo * 0.9, min_velo)
    elif np.radians(4) <= abs(steering()) < np.radians(5):
        velo_new = velo
    else:
        velo_new = min(velo * 1.1, max_velo)

    return velo_new


def plot_arrow(x, y, alpha, length=1, width=1, fc="r", ec="k"):     # 차량 plot

    plt.arrow(x, y, length * np.cos(alpha), length * np.sin(alpha+np.pi),
              fc=fc, ec=ec, head_width=width, head_length=width, zorder=10)
    plt.plot(x, y)


def append_list():

    list_time.append(time)
    list_steering.append(np.degrees(steering()))
    list_ctr.append(cross_track_error())
    list_velo.append(velo)


print("Stanley Method path tracking simulation start\n")

road_x, road_y = make_road()

list_time = []
list_steering = []
list_ctr = []
list_velo = []

# output
for time in np.arange(0.0, 50, time_tic):

    yaw += steering()                         # heading update
    front_x += velo * time_tic * np.cos(abs(yaw))  # x 위치 좌표 update
    front_y -= velo * time_tic * np.sin(yaw)  # y 위치 좌표 update
    rear_x = front_x - L * np.cos(abs(yaw))
    rear_y = front_y + L * np.sin(yaw)

    velo = update_velo()
    append_list()

# 차량 주행 시뮬레이션
    plt.cla()
    plt.grid(True)
    plt.title('Stanley Method')
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.plot(road_x, road_y, '-')
    plt.axis([start_x, end_x, -20, 20])
    plot_arrow(rear_x, rear_y, yaw)
    plt.pause(0.0001)

    if abs(end_x - front_x) < 0.5 and abs(front_y) < 0.5:  # 목적지 도달 break
        plt.pause(0.5)
        print("목적지 도달")
        break

    elif cross_track_error() > 10:          # 목적지 도달 못함 break
        print("목적지 도달 실패")
        break

fig, axs = plt.subplots(3, 1)
fig.subplots_adjust(hspace=1)

# Cross track error, time graph
axs[0].plot(list_time, list_ctr, lw=1)
axs[0].set_title("Cross track error")
axs[0].axis([0, time, 0, 2.0])
axs[0].set_xlabel("Time[s]")
axs[0].set_ylabel("[m]")
axs[0].grid(True)

# Steering angle, time graph
axs[1].plot(list_time, list_steering, lw=1)
axs[1].set_title("Steering angle")
axs[1].set_xlabel("Time[s]")
axs[1].set_ylabel("[degrees]")
axs[1].axis([0, time, -np.degrees(max_steering)-5, np.degrees(max_steering)+5])
axs[1].grid(True)

# Velo, time graph
axs[2].plot(list_time, list_velo, lw=1)
axs[2].set_title("Velo")
axs[2].axis([0, time, 0, max_velo+1])
axs[2].set_xlabel("Time[s]")
axs[2].set_ylabel("[m/s]")
axs[2].grid(True)

plt.show()
