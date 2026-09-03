from IMU import IMU_yaw
import math
import lagrange
import leg
from controller import Motor, PositionSensor, Gyro, Accelerometer, Robot, InertialUnit
import mymath
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gyro = Gyro("gyro")
imu = InertialUnit("imu")
accelerometer = Accelerometer("accelerometer")
robot_yaw = IMU_yaw()

imu.enable(timestep)
gyro.enable(timestep)
accelerometer.enable(timestep)


# 髋电机
motor_1 = Motor("Left_Front_Motor")  # 左腿第一个主动关节
motor_3 = Motor("Left_Back_Motor")
motor_2 = Motor("Right_Front_Motor")  # 右腿第一个主动关节
motor_4 = Motor("Right_Back_Motor")

# 足电机
motor_5 = Motor("Left_Wheel")  # 安装反的，设置力矩要加负号
motor_6 = Motor("Right_Wheel")  # 安装反的，设置力矩要加负号

# 髋角度
ps_1 = PositionSensor("Left_Front_Motor_Sensor")
ps_3 = PositionSensor("Left_Back_Motor_Sensor")
ps_2 = PositionSensor("Right_Front_Motor_Sensor")
ps_4 = PositionSensor("Right_Back_Motor_Sensor")

# 足角度
ps_5 = PositionSensor("Left_Wheel_Sensor")  # 安装反的，读取数据要加负号
ps_6 = PositionSensor("Right_Wheel_Sensor")  # 安装反的，读取数据要加负号


# 初始化电机
for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
    motor.setPosition(float("inf"))
    motor.setVelocity(0.0)

# 初始化传感器
for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, accelerometer, imu]:
    sensor.enable(timestep)


# 差分初始化
d_t = timestep / 1000
Theta_b = mymath.Discreteness(d_t)

diff_yaw = mymath.Discreteness(d_t)
Theta_wl = mymath.Discreteness(d_t)
Theta_wr = mymath.Discreteness(d_t)
Roll = mymath.Discreteness(d_t)

theta_l1 = mymath.Discreteness(d_t)
theta_l4 = mymath.Discreteness(d_t)
theta_r1 = mymath.Discreteness(d_t)
theta_r4 = mymath.Discreteness(d_t)

d_Ll = mymath.Discreteness(d_t)
d_Lr = mymath.Discreteness(d_t)

Ll = mymath.Discreteness(d_t)

Ll.last_diff = 0.16000280

theta_l1.last_diff = 3.03552063
theta_r1.last_diff = 3.03552063
theta_l4.last_diff = 0.10607202
theta_r4.last_diff = 0.10607202

target_L0_l = 0.30
target_L0_r = 0.30

counter = 0
print_interval = max(1, int(100 / timestep))  # 每100ms打印一次

# 左腿两个电机采用位置控制
# motor_1.setVelocity(0.3)
# motor_3.setVelocity(0.3)
# motor_2.setVelocity(0.3)
# motor_4.setVelocity(0.3)
# 绝对目标位置都是1 rad
# motor_1.setPosition(1.57)
# motor_3.setPosition(1.57)
# motor_2.setPosition(1.57)
# motor_4.setPosition(1.57)


# PID初始化
F0_control_l = mymath.PID_control(400, 0, 4000, target_L0_l)
F0_control_r = mymath.PID_control(400, 0, 4000, target_L0_r)

# 轮子半径
r = 0.06
s = 0.0

current_time = 0

while robot.step(timestep) != -1:
    current_time = current_time + d_t

    # 获取姿态角
    roll, pitch, yaw = imu.getRollPitchYaw()
    theta_b = pitch
    yaw = robot_yaw.round_yaw(yaw)

    dot_roll, dot_theta_b, dot_yaw = gyro.getValues()
    ax, ay, az = accelerometer.getValues()

    phi_l1 = 3.03552063 - ps_1.getValue()
    phi_l4 = 0.10607202 - ps_3.getValue()

    phi_r1 = 3.03552063 - ps_2.getValue()
    phi_r4 = 0.10607202 - ps_4.getValue()

    phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
    phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

    theta_ll = 1.570796 - phi0_l + theta_b
    theta_lr = 1.570796 - phi0_r + theta_b

    theta_wl = -ps_5.getValue()
    theta_wr = -ps_6.getValue()

    dot_theta_wl = Theta_wl.Diff(theta_wl)
    dot_theta_wr = Theta_wr.Diff(theta_wr)

    # 四个主动关节角速度，单位 rad/s
    dot_phi_l1 = theta_l1.Diff(phi_l1)
    dot_phi_l4 = theta_l4.Diff(phi_l4)
    dot_phi_r1 = theta_r1.Diff(phi_r1)
    dot_phi_r4 = theta_r4.Diff(phi_r4)

    # leg.spd()第二个返回值是 -dot_phi0
    # 也就是虚拟腿相对机身的摆动角速度
    dot_L0_l, dot_Phi0_l_negative = leg.spd(
        dot_phi_l1,
        dot_phi_l4,
        0.21,
        0.25,
        0.25,
        0.21,
        0.0,
        phi_l1,
        phi_l4,
    )

    dot_L0_r, dot_Phi0_r_negative = leg.spd(
        dot_phi_r1,
        dot_phi_r4,
        0.21,
        0.25,
        0.25,
        0.21,
        0.0,
        phi_r1,
        phi_r4,
    )

    # 腿长伸缩加速度
    ddot_L0_l = d_Ll.Diff(dot_L0_l)
    ddot_L0_r = d_Lr.Diff(dot_L0_r)

    dot_theta_ll = dot_Phi0_l_negative + dot_theta_b
    dot_theta_lr = dot_Phi0_r_negative + dot_theta_b

    # 位移与速度
    # s = r * (theta_wl + theta_wr) / 2
    dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
    dot_s_b = (
        dot_s_wheel
        + 0.5
        * (
            L0_l * dot_theta_ll * math.cos(theta_ll)
            + L0_r * dot_theta_lr * math.cos(theta_lr)
        )
        + 0.5 * (dot_L0_l * math.sin(theta_ll) + dot_L0_r * math.sin(theta_lr))
    )
    s += dot_s_b * d_t

    # 当前的状态
    current_state = np.matrix(
        [
            [s],
            [dot_s_b],
            [yaw],
            [dot_yaw],
            [theta_ll],
            [dot_theta_ll],
            [theta_lr],
            [dot_theta_lr],
            [theta_b],
            [dot_theta_b],
        ]
    )

    # 计算虚拟腿力到两个髋电机力矩的转换矩阵
    if 0 < current_time <= 10000:
        expect_state = np.matrix([[0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])
    else:
        break
    K = lagrange.K(target_L0_l, target_L0_r,50,1,500,1,500,1,500,1,5000,1,1,1,1,1)

    U = K * (expect_state - current_state)

    T_wl = float(U[0, 0])
    T_wr = float(U[1, 0])
    T_ll = float(U[2, 0])
    T_lr = float(U[3, 0])

    #VMC关节电机映射                              
    JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
    JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

    # 腿长PID
    pid_force_l = F0_control_l.position_pid(L0_l)
    pid_force_r = F0_control_r.position_pid(L0_r)

    F_l = -pid_force_l
    F_r = -pid_force_r
    # 第二项为虚拟腿摆动力矩，本次只测腿长，所以设为0
    T_JOINT_L = JRM_L * np.matrix(
        [
            [F_l],
            [0.0],
        ]
    )

    T_JOINT_R = JRM_R * np.matrix(
        [
            [F_r],
            [0.00],
        ]
    )

    T1_l = float(T_JOINT_L[0, 0])
    T1_r = float(T_JOINT_L[1, 0])
    T2_l = float(T_JOINT_R[0, 0])
    T2_r = float(T_JOINT_R[1, 0])

    motor_1.setTorque(T1_l)
    motor_3.setTorque(T1_r)
    motor_2.setTorque(T2_l)
    motor_4.setTorque(T2_r)

    counter += 1
    if counter >= print_interval:
        counter = 0

    print(
        f"theta_b={math.degrees(theta_b):+.2f}°, "
        f"dot_theta_b={math.degrees(dot_theta_b):+.2f}°/s"
    )

    print(
        f"左腿：L0={L0_l:.4f} m, "
        f"theta_ll={math.degrees(theta_ll):+.2f}°, "
        f"dot_theta_ll={math.degrees(dot_theta_ll):+.2f}°/s"
    )

    print(
        f"右腿：L0={L0_r:.4f} m, "
        f"theta_lr={math.degrees(theta_lr):+.2f}°, "
        f"dot_theta_lr={math.degrees(dot_theta_lr):+.2f}°/s"
    )

    print(
        f"s={s:+.4f} m, "
        f"dot_s_b={dot_s_b:+.4f} m/s, "
        f"yaw={math.degrees(yaw):+.2f}°, "
        f"dot_yaw={math.degrees(dot_yaw):+.2f}°/s"
    )

    print("--------------------------------")
