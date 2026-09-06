# from IMU import IMU_yaw
# import math
# import lagrange
# import leg
# from controller import Motor, PositionSensor, Gyro, Accelerometer, Robot, InertialUnit
# import mymath
# import numpy as np

# robot = Robot()
# timestep = int(robot.getBasicTimeStep())

# gyro = Gyro("gyro")
# imu = InertialUnit("imu")
# accelerometer = Accelerometer("accelerometer")
# robot_yaw = IMU_yaw()

# imu.enable(timestep)
# gyro.enable(timestep)
# accelerometer.enable(timestep)


# # 髋电机
# motor_1 = Motor("Left_Front_Motor")  # 左腿第一个主动关节
# motor_3 = Motor("Left_Back_Motor")
# motor_2 = Motor("Right_Front_Motor")  # 右腿第一个主动关节
# motor_4 = Motor("Right_Back_Motor")

# # 足电机
# motor_5 = Motor("Left_Wheel")  # 安装反的，设置力矩要加负号
# motor_6 = Motor("Right_Wheel")  # 安装反的，设置力矩要加负号

# # 髋角度
# ps_1 = PositionSensor("Left_Front_Motor_Sensor")
# ps_3 = PositionSensor("Left_Back_Motor_Sensor")
# ps_2 = PositionSensor("Right_Front_Motor_Sensor")
# ps_4 = PositionSensor("Right_Back_Motor_Sensor")

# # 足角度
# ps_5 = PositionSensor("Left_Wheel_Sensor")  # 安装反的，读取数据要加负号
# ps_6 = PositionSensor("Right_Wheel_Sensor")  # 安装反的，读取数据要加负号


# # 初始化电机
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setPosition(0.0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, accelerometer, imu]:
#     sensor.enable(timestep)


# # 差分初始化
# d_t = timestep / 1000
# Theta_b = mymath.Discreteness(d_t)

# diff_yaw = mymath.Discreteness(d_t)
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# Roll = mymath.Discreteness(d_t)

# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# d_Ll = mymath.Discreteness(d_t)
# d_Lr = mymath.Discreteness(d_t)

# Ll = mymath.Discreteness(d_t)

# Ll.last_diff = 0.16000280

# theta_l1.last_diff = 3.03552063
# theta_r1.last_diff = 3.03552063
# theta_l4.last_diff = 0.10607202
# theta_r4.last_diff = 0.10607202

# target_L0_l = 0.38
# target_L0_r = 0.38

# counter = 0
# print_interval = max(1, int(100 / timestep))  # 每100ms打印一次


# # PID初始化
# F0_control_l = mymath.PID_control(400, 0, 4000, target_L0_l)
# F0_control_r = mymath.PID_control(400, 0, 4000, target_L0_r)

# # 轮子半径
# r = 0.06
# s = 0.0

# # 左腿两个电机采用位置控制
# motor_1.setVelocity(0.2)
# motor_3.setVelocity(0.2)
# motor_2.setVelocity(0.2)
# motor_4.setVelocity(0.2)
# # 绝对目标位置都是1 rad
# motor_1.setPosition(0.3)
# motor_3.setPosition(0.3)
# motor_2.setPosition(0.3)
# motor_4.setPosition(0.3)

# current_time = 0
# flag = 0
# expect_state = np.zeros((10, 1))

# while robot.step(timestep) != -1:
#     current_time = current_time + d_t

#     # 获取姿态角
#     roll, pitch, yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(yaw)

#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()
#     ax, ay, az = accelerometer.getValues()

#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()

#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = 1.570796 - phi0_l + theta_b
#     theta_lr = 1.570796 - phi0_r + theta_b

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()

#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     # 四个主动关节角速度，单位 rad/s
#     dot_phi_l1 = theta_l1.Diff(phi_l1)
#     dot_phi_l4 = theta_l4.Diff(phi_l4)
#     dot_phi_r1 = theta_r1.Diff(phi_r1)
#     dot_phi_r4 = theta_r4.Diff(phi_r4)

#     # leg.spd()第二个返回值是 -dot_phi0
#     # 也就是虚拟腿相对机身的摆动角速度
#     dot_L0_l, dot_Phi0_l_negative = leg.spd(
#         dot_phi_l1,
#         dot_phi_l4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_l1,
#         phi_l4,
#     )

#     dot_L0_r, dot_Phi0_r_negative = leg.spd(
#         dot_phi_r1,
#         dot_phi_r4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_r1,
#         phi_r4,
#     )

#     # 腿长伸缩加速度
#     ddot_L0_l = d_Ll.Diff(dot_L0_l)
#     ddot_L0_r = d_Lr.Diff(dot_L0_r)

#     dot_theta_ll = dot_Phi0_l_negative + dot_theta_b
#     dot_theta_lr = dot_Phi0_r_negative + dot_theta_b

#     # 位移与速度
#     # s = r * (theta_wl + theta_wr) / 2
#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_b = (
#         dot_s_wheel
#         + 0.5
#         * (
#             L0_l * dot_theta_ll * math.cos(theta_ll)
#             + L0_r * dot_theta_lr * math.cos(theta_lr)
#         )
#         + 0.5 * (dot_L0_l * math.sin(theta_ll) + dot_L0_r * math.sin(theta_lr))
#     )
#     s += dot_s_b * d_t

#     # 当前的状态
#     # current_state = np.matrix(
#     #     [
#     #         [s],
#     #         [dot_s_b],
#     #         [yaw],
#     #         [dot_yaw],
#     #         [theta_ll],
#     #         [dot_theta_ll],
#     #         [theta_lr],
#     #         [dot_theta_lr],
#     #         [theta_b],
#     #         [dot_theta_b],
#     #     ]
#     # )
#     current_state = np.matrix(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ]
#     )

#     # 计算虚拟腿力到两个髋电机力矩的转换矩阵
#     if 0 < current_time <= 3:
#         flag = 0
#     elif 3 < current_time < 1000:
#         expect_state = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
#         flag = 1
#     else:
#         break
#     K = lagrange.K(
#         target_L0_l, target_L0_r, 50, 1, 500, 1, 500, 1, 500, 1, 5000, 1, 1, 1, 1, 1
#     )

#     U = K * (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     T_wl = 0
#     T_wr = 0

#     # VMC关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长PID
#     pid_force_l = F0_control_l.position_pid(L0_l)
#     pid_force_r = F0_control_r.position_pid(L0_r)

#     F_l = -pid_force_l
#     F_r = -pid_force_r
#     # 第二项为虚拟腿摆动力矩，本次只测腿长，所以设为0
#     T_JOINT_L = JRM_L * np.matrix(
#         [
#             [F_l],
#             # [0],
#             # [T_ll],
#             [0],
#         ]
#     )

#     T_JOINT_R = JRM_R * np.matrix(
#         [
#             [F_r],
#             # [0],
#             # [T_lr],
#             [0],
#         ]
#     )

#     T1_l = float(T_JOINT_L[0, 0])
#     T1_r = float(T_JOINT_L[1, 0])
#     T2_l = float(T_JOINT_R[0, 0])
#     T2_r = float(T_JOINT_R[1, 0])

#     motor_1.setTorque(float(np.clip(T1_l, -25.0, 25.0)))
#     motor_3.setTorque(float(np.clip(T1_r, -25.0, 25.0)))
#     motor_2.setTorque(float(np.clip(T2_l, -25.0, 25.0)))
#     motor_4.setTorque(float(np.clip(T2_r, -25.0, 25.0)))
#     motor_5.setTorque(float(np.clip(-T_wl, -10.0, 10.0)))
#     motor_6.setTorque(float(np.clip(-T_wr, -10.0, 10.0)))

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         print(
#             f"theta_b={math.degrees(theta_b):+.2f}°, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f}°/s"
#         )

#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"theta_ll={math.degrees(theta_ll):+.2f}°, "
#             f"phi_l1={phi_l1:+.2f}rad, "
#             f"phi_l4={phi_l4:+.2f}rad, "
#             f"ps_1={ps_1.getValue():+.2f}rad, "
#             f"ps_4={ps_4.getValue():+.2f}rad, "
#             f"dot_theta_ll={math.degrees(dot_theta_ll):+.2f}°/s,"
#             f"T1_l={T1_l}N, "
#             f"T1_r={T1_r}N"
#         )

#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"theta_lr={math.degrees(theta_lr):+.2f}°, "
#             f"dot_theta_lr={math.degrees(dot_theta_lr):+.2f}°/s"
#             f"T2_l={T2_l}N, "
#             f"T2_r={T2_r}N"
#         )

#         print(
#             f"phi0_l={math.degrees(phi0_l):+.4f} °, "
#             f"phi0_r={math.degrees(phi0_r):+.4f} °, "
#             f"flag={flag} , "
#             f"pid_force_l={pid_force_l:+.2f}N, "
#             f"pid_force_r={pid_force_r:+.2f}N"
#         )

#         print("--------------------------------")


















# """Webots 五连杆腿摆 LQR 极性测试。

# 测试流程：
# 1. 0～3 秒：四个髋电机使用位置控制，让左右虚拟腿同时偏转。
# 2. 3 秒以后：髋电机只切换一次到力矩控制，由 LQR 使腿摆角回到 0。
# 3. 同时以小力矩缓慢启用两个轮子，用于验证轮子输出极性。

# 运行本测试前，请在 Webots 中把 Robot 根节点设置为 locked TRUE。
# """

# import math

# import numpy as np
# from controller import Robot

# from IMU import IMU_yaw
# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# PERTURB_TIME = 3.0                 # 前 3 秒制造腿摆偏角
# PERTURB_MOTOR_POSITION = 0.50      # rad，约 17.2 度
# POSITION_SPEED = 0.20              # rad/s

# # 极性测试必须从小力矩开始，确认方向正确后再逐步增大。
# HIP_TORQUE_LIMIT = 0.30            # N*m

# # 当前测试没有启用腿长 PID，腿长仍接近初始的 0.16 m。
# LQR_MODEL_LEG_LENGTH = 0.16000280  # m

# # 已确认髋关节腿摆极性后，启用轮子做第二阶段测试。
# ENABLE_WHEEL_TORQUE = True
# WHEEL_TORQUE_LIMIT = 0.10          # N*m
# WHEEL_RAMP_TIME = 1.0              # s，轮力矩在 1 秒内由 0 缓慢增加


# robot = Robot()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)

# robot_yaw = IMU_yaw()


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)

# theta_l1_diff.last_diff = 3.03552063
# theta_r1_diff.last_diff = 3.03552063
# theta_l4_diff.last_diff = 0.10607202
# theta_r4_diff.last_diff = 0.10607202


# # ----------------------- 0～3 秒位置控制 -----------------------
# # 这里只发送位置命令。此阶段不能再调用四个髋电机的 setTorque()。
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(POSITION_SPEED)

# # 左右腿同时向同一腿摆方向偏转。
# motor_1.setPosition(PERTURB_MOTOR_POSITION)
# motor_3.setPosition(PERTURB_MOTOR_POSITION)
# motor_2.setPosition(PERTURB_MOTOR_POSITION)
# motor_4.setPosition(PERTURB_MOTOR_POSITION)

# # 轮子一直使用力矩模式。
# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# # K 只计算一次，避免每个仿真周期重复求解 Riccati 方程。
# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print("控制器启动：前 3 秒位置控制，3 秒后切换到腿摆 LQR。", flush=True)


# while robot.step(timestep) != -1:
#     current_time = robot.getTime()

#     # IMU：模型 x 向前、z 向上时，pitch/绕 y 轴角速度对应机体俯仰。
#     roll, pitch, raw_yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(raw_yaw)
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 主动关节角转换到五连杆运动学角度。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     # leg.spd() 的第二个输出是 -dot(phi0)。
#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 保留你当前模型的轮子坐标约定：控制坐标 = -PositionSensor 原始值。
#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 本测试假设其余 6 个状态为 0，只把两条腿的摆角和角速度送入 LQR。
#     current_state = np.array(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ],
#         dtype=float,
#     )

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 把虚拟腿摆力矩映射到四个髋关节电机。
#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     # Webots 原始轮电机命令。模型轮力矩与电机命令之间保留一个负号。
#     wheel_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if current_time < PERTURB_TIME:
#         phase = "POSITION"

#         # 关键：这里不能调用 motor_1～motor_4.setTorque()。
#         # 四个髋电机会继续完成循环外设置的 0.30 rad 位置目标。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     else:
#         phase = "LQR"

#         # 到达 3 秒时只执行一次位置模式 -> 力矩模式切换。
#         if not released:
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             released = True
#             print("3 秒到达：已释放位置控制，开始 LQR 回中。", flush=True)

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)

#         if ENABLE_WHEEL_TORQUE:
#             # 切换后缓慢投入轮力矩，防止瞬间冲击。
#             wheel_ramp = float(
#                 np.clip((current_time - PERTURB_TIME) / WHEEL_RAMP_TIME, 0.0, 1.0)
#             )
#             cmd_wl = float(
#                 np.clip(T_wl * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             motor_5.setTorque(cmd_wl)
#             motor_6.setTorque(cmd_wr)
#         else:
#             motor_5.setTorque(0.0)
#             motor_6.setTorque(0.0)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "LQR":
#             left_trend = "回中" if theta_ll * dot_theta_ll < 0.0 else "离开/暂未运动"
#             right_trend = "回中" if theta_lr * dot_theta_lr < 0.0 else "离开/暂未运动"

#             if abs(T_wl) < 1e-4:
#                 left_wheel_polarity = "力矩接近0"
#             elif T_wl * dot_theta_wl > 0.0:
#                 left_wheel_polarity = "极性一致"
#             else:
#                 left_wheel_polarity = "反向/暂未转动"

#             if abs(T_wr) < 1e-4:
#                 right_wheel_polarity = "力矩接近0"
#             elif T_wr * dot_theta_wr > 0.0:
#                 right_wheel_polarity = "极性一致"
#             else:
#                 right_wheel_polarity = "反向/暂未转动"
#         else:
#             left_trend = "制造偏角"
#             right_trend = "制造偏角"
#             left_wheel_polarity = "尚未启用"
#             right_wheel_polarity = "尚未启用"

#         print(
#             f"t={current_time:5.2f}s  phase={phase}  "
#             f"theta_b={math.degrees(theta_b):+.2f} deg"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"Phi0={math.degrees(phi0_l):+.2f} deg, "
#             f"theta={theta_ll:+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s, "
#             f"phi_l1={phi_l1:+.2f} deg, "
#             f"phi_l4={phi_l4:+.2f} deg, "
#             f"判断={left_trend}"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"Phi0={math.degrees(phi0_r):+.2f} deg, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s, "
#             f"phi_r1={math.degrees(phi_r1):+.2f} deg, "
#             f"phi_r4={math.degrees(phi_r4):+.2f} deg, "
#             f"判断={right_trend}"
#         )
#         print(
#             f"LQR：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"髋电机命令：M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"左轮：模型速度={dot_theta_wl:+.4f} rad/s, "
#             f"电机命令={cmd_wl:+.4f} N*m, 判断={left_wheel_polarity}"
#         )
#         print(
#             f"右轮：模型速度={dot_theta_wr:+.4f} rad/s, "
#             f"电机命令={cmd_wr:+.4f} N*m, 判断={right_wheel_polarity}, "
#             f"ramp={wheel_ramp:.2f}"
#         )
#         print("--------------------------------")










# """Webots 机体俯仰 LQR 极性测试。

# 状态向量中只有 theta_b、dot_theta_b 使用真实测量值，其余 8 个状态全部为 0。

# 通过 TEST_MODE 分别测试：
# - "LEG_ONLY"：只施加虚拟腿摆力矩，两个轮子的力矩严格为 0；
# - "WHEEL_ONLY"：只施加轮力矩，四个髋关节电机的力矩严格为 0；
# - "BOTH"：腿摆与轮子同时工作，仅用于前两项通过后的组合验证。

# 流程：
# 1. 0～3 秒：Supervisor 固定 Robot 根节点，并将机体设置为 5 度初始倾角；
#    四个髋关节保持零位，轮力矩为 0。
# 2. 3 秒以后：解锁 Robot，髋关节切换到力矩模式，由机体状态产生的
#    LQR 轮力矩和虚拟腿摆力矩开始工作。

# WBT 使用要求：
# - Robot 节点 supervisor TRUE
# - 轮子接触地面
# - Robot 的 locked 初始值 TRUE/FALSE 均可，代码会自动控制
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次运行选 "LEG_ONLY"，复位仿真后改成 "WHEEL_ONLY" 再运行一次。
# TEST_MODE = "BOTH"
# VALID_TEST_MODES = ("LEG_ONLY", "WHEEL_ONLY", "BOTH")

# if TEST_MODE not in VALID_TEST_MODES:
#     raise ValueError(f"TEST_MODE 必须是 {VALID_TEST_MODES} 之一")

# RELEASE_TIME = 3.0
# INITIAL_BODY_PITCH = math.radians(-5.0)

# LQR_MODEL_LEG_LENGTH = 0.16000280

# HIP_POSITION_SPEED = 0.20
# HIP_TORQUE_LIMIT = 0.30
# WHEEL_TORQUE_LIMIT = 0.10
# TORQUE_RAMP_TIME = 0.50

# # 倾角超过该值就关闭全部力矩并重新固定机体，避免极性错误时摔飞。
# SAFETY_PITCH = math.radians(25.0)


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请把 Robot 的 supervisor 设置为 TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")

# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先固定机体，再设置绕 y 轴的初始俯仰角。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # ----------------------- 0～3 秒保持腿零位 -----------------------
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(HIP_POSITION_SPEED)
#     hip_motor.setPosition(0.0)

# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- 轮速差分 ---------------------------
# # 保留你当前使用的轮子反馈坐标：theta_w = -PositionSensor。
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print(
#     f"控制器启动：TEST_MODE={TEST_MODE}，机体固定在 5 度，"
#     "3 秒后释放并启动仅机体状态的 LQR。",
#     flush=True,
# )

# controller_start_time = robot.getTime()


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 运动学只用于把 LQR 的虚拟腿摆力矩映射到四个髋电机。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 10 维状态顺序：
#     # s, ds, yaw, dyaw, theta_ll, dtheta_ll,
#     # theta_lr, dtheta_lr, theta_b, dtheta_b
#     #
#     # 本测试只有最后两个机体状态不是 0。
#     current_state = np.zeros((10, 1))
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     # 只输入虚拟腿摆力矩，轴向力暂时为 0。
#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     torque_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if elapsed_time < RELEASE_TIME:
#         phase = "HOLD"

#         # 髋电机继续执行零位位置控制；此处不能调用髋电机 setTorque()。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     elif not safety_stopped:
#         phase = "BODY_LQR"

#         if not released:
#             # 髋关节切换到力矩模式。
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             # 清除固定阶段的物理速度后释放机体。
#             robot_node.resetPhysics()
#             locked_field.setSFBool(False)
#             released = True

#             print("3 秒到达：机体已释放，启动机体俯仰 LQR。", flush=True)

#         torque_ramp = float(
#             np.clip((elapsed_time - RELEASE_TIME) / TORQUE_RAMP_TIME, 0.0, 1.0)
#         )

#         # 根据 TEST_MODE 只允许被测执行器产生实际力矩。
#         if TEST_MODE in ("LEG_ONLY", "BOTH"):
#             cmd_m1 *= torque_ramp
#             cmd_m3 *= torque_ramp
#             cmd_m2 *= torque_ramp
#             cmd_m4 *= torque_ramp
#         else:
#             # WHEEL_ONLY：髋关节已经进入力矩模式，但实际命令严格为 0。
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0

#         if TEST_MODE in ("WHEEL_ONLY", "BOTH"):
#             # 已按你的实测结果修正：LQR轮力矩和Webots电机命令同号。
#             cmd_wl = float(
#                 np.clip(T_wl * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#         else:
#             # LEG_ONLY：轮子保持力矩模式，但实际命令严格为 0。
#             cmd_wl = 0.0
#             cmd_wr = 0.0

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#         if abs(theta_b) > SAFETY_PITCH:
#             for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#                 motor.setTorque(0.0)

#             locked_field.setSFBool(True)
#             safety_stopped = True
#             phase = "SAFETY_STOP"
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0
#             cmd_wl = 0.0
#             cmd_wr = 0.0
#             print("安全停止：机体倾角超过 25 度，请检查极性。", flush=True)

#     else:
#         phase = "SAFETY_STOP"

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "BODY_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 body_trend = "接近中心"
#             elif theta_b * dot_theta_b < 0.0:
#                 body_trend = "正在回正"
#             else:
#                 body_trend = "离开中心/暂未运动"
#         elif phase == "HOLD":
#             body_trend = "保持初始倾角"
#         else:
#             body_trend = "已停止"

#         print(
#             f"t={elapsed_time:5.2f}s, mode={TEST_MODE}, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"判断={body_trend}"
#         )
#         print(
#             "输入状态："
#             f"theta_b={current_state[8, 0]:+.5f} rad, "
#             f"dot_theta_b={current_state[9, 0]:+.5f} rad/s，"
#             "其余状态全部为0"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"实际命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f}, "
#             f"M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"轮速：左={dot_theta_wl:+.3f}, 右={dot_theta_wr:+.3f} rad/s, "
#             f"ramp={torque_ramp:.2f}"
#         )
#         print("--------------------------------")


















# """Webots 轮腿机器人 6 状态初步平衡测试。

# LQR 中仅使用以下 6 个真实状态：
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# s、dot_s、yaw、dot_yaw 全部置 0。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# # TARGET_LEG_LENGTH_L = 0.16000280
# # TARGET_LEG_LENGTH_R = 0.16000280
# TARGET_LEG_LENGTH_L = 0.2000280
# TARGET_LEG_LENGTH_R = 0.2000280

# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# WHEEL_TORQUE_LIMIT = 10.10    # N*m，第一次平衡测试从小力矩开始

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 6 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 10维向量中只有腿摆4状态和机体2状态是真实值。
#     # s、dot_s、yaw、dot_yaw 全部为0。
#     current_state = np.zeros((10, 1))
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "SIX_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [theta_b, dot_theta_b, theta_ll, theta_lr, L0_l, L0_r]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "SIX_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "6状态输入："
#             f"[{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# TARGET_LEG_LENGTH_L = 0.16000280
# TARGET_LEG_LENGTH_R = 0.16000280
# # TARGET_LEG_LENGTH_L = 0.20
# # TARGET_LEG_LENGTH_R = 0.20


# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     500, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡与变腿长测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 8 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到 0.20 m，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 8 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.220
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")


# """完整 10 状态轮腿平衡与变腿长测试。

# LQR 中使用以下 10 个真实状态：
#     s, dot_s, yaw, dot_yaw,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# 启动时的实际朝向作为 yaw=0，原来已经验证的腿长 PD、VMC 映射
# 以及轮子和髋关节输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 10 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到设定长度，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 10 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 腿长到达终点后再原地转向 +pi/2。yaw 目标也使用 smoothstep，避免
# # 直接给 90 度阶跃造成左右轮差动力矩瞬间饱和。
# FINAL_YAW = math.pi / 2.0
# YAW_RAMP_TIME = 4.0

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0
# YAW_HOLD_TIME = LEG_LENGTH_HOLD_TIME + LEG_LENGTH_RAMP_TIME

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 完整状态首次联调仍保留轮力矩限幅，避免偏航或姿态误差导致瞬时冲击。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_YAW = math.radians(120.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.240
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     1000, 5,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、偏航角、腿摆角、轮角以及世界位置全部作为零点。
# _, theta_b_init, yaw_init = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与完整 10 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 5 秒前保持原朝向，5～9 秒平滑转到 +pi/2，之后保持 90 度。
#     yaw_linear_ratio = float(np.clip(
#         (elapsed_time - YAW_HOLD_TIME) / YAW_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     yaw_ramp_ratio = (
#         yaw_linear_ratio * yaw_linear_ratio * (3.0 - 2.0 * yaw_linear_ratio)
#     )
#     target_yaw = FINAL_YAW * yaw_ramp_ratio

#     if elapsed_time < YAW_HOLD_TIME:
#         yaw_phase = "HOLD"
#     elif yaw_linear_ratio < 1.0:
#         yaw_phase = "TURN"
#     else:
#         yaw_phase = "FINAL"

#     expect_state[2, 0] = target_yaw

#     # 机体姿态。yaw 使用启动时朝向作为 0，并包角到 [-pi, pi]。
#     roll, theta_b, raw_yaw = imu.getRollPitchYaw()
#     yaw = math.atan2(
#         math.sin(raw_yaw - yaw_init),
#         math.cos(raw_yaw - yaw_init),
#     )
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 完整 10 状态，顺序必须与 lagrange.py 建模顺序完全一致。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[2, 0] = yaw
#     current_state[3, 0] = dot_yaw
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "TEN_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world, yaw, dot_yaw,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or abs(yaw) > SAFETY_YAW
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if abs(yaw) > SAFETY_YAW:
#             stop_reasons.append("偏航角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "TEN_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"偏航计划：{yaw_phase}, ramp={yaw_ramp_ratio:.3f}, "
#             f"target={math.degrees(target_yaw):+.2f} deg"
#         )
#         print(
#             f"偏航反馈：yaw={math.degrees(yaw):+.2f} deg, "
#             f"error={math.degrees(target_yaw - yaw):+.2f} deg, "
#             f"dot_yaw={math.degrees(dot_yaw):+.2f} deg/s"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "10状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{yaw:+.4f}, {dot_yaw:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"轮力矩分量：平均={(T_wl + T_wr) / 2.0:+.4f}, "
#             f"差动={(T_wr - T_wl) / 2.0:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")



























# # -*- coding: utf-8 -*-
# # my_controller.py：10 状态 LQR + 腿长 PD

# import math
# import numpy as np

# import lagrange
# import leg
# import mymath
# from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit

# robot = Supervisor()  # Webots 的 Robot 节点需设置 supervisor TRUE
# timestep = int(robot.getBasicTimeStep())
# d_t = timestep / 1000

# # 初始机体倾角
# initial_body_pitch = math.radians(0.0)
# robot_node = robot.getSelf()
# locked_field = robot_node.getField('locked')
# locked_field.setSFBool(True)
# robot_node.getField('rotation').setSFRotation([0, 1, 0, initial_body_pitch])
# robot_node.resetPhysics()

# # 足电机
# motor_5 = Motor('Left_Wheel')
# motor_6 = Motor('Right_Wheel')

# # 髋电机
# motor_3 = Motor('Left_Front_Motor')   # 左后
# motor_1 = Motor('Left_Back_Motor')  # 左前
# motor_4 = Motor('Right_Front_Motor')  # 右后
# motor_2 = Motor('Right_Back_Motor') # 右前


# # 足角度
# ps_5 = PositionSensor('Left_Wheel_Sensor')
# ps_6 = PositionSensor('Right_Wheel_Sensor')

# # 髋角度
# ps_1 = PositionSensor('Left_Back_Motor_Sensor')
# ps_3 = PositionSensor('Left_Front_Motor_Sensor')
# ps_2 = PositionSensor('Right_Back_Motor_Sensor')
# ps_4 = PositionSensor('Right_Front_Motor_Sensor')

# gyro = Gyro('gyro')
# imu = InertialUnit('imu')

# # 初始化电机：直接力矩控制
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float('inf'))
#     motor.setVelocity(0)
#     motor.setTorque(0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, imu]:
#     sensor.enable(timestep)

# # 差分初始化
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# # 腿长目标：先保持 2 秒，再用 0.5 秒从 0.16 m 平滑变到 0.22 m
# start_L0 = 0.16000280
# final_L0 = 0.22
# leg_hold_time = 2.0
# leg_change_time = 0.5

# # 腿长 PD 参数与支撑力前馈
# leg_kp = 400.0
# leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
# gravity_force_l = 22.0        # N
# gravity_force_r = 22.0        # N

# # 输出限幅
# max_leg_force = 40.0         # N
# max_hip_torque = 10.0        # N*m
# max_wheel_torque = 0.50      # N*m
# r = 0.05995                  # 轮半径，m

# # 两组 K 只在启动时计算，之后随腿长目标插值
# K_start = lagrange.K(start_L0, start_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)
# K_final = lagrange.K(final_L0, final_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)

# s_local = 0.0
# # 采集一拍真实初值，避免差分器第一拍产生虚假速度
# if robot.step(timestep) == -1:
#     raise RuntimeError('仿真在控制器初始化时结束')

# phi_l1 = 3.03552063 - ps_1.getValue()
# phi_l4 = 0.10607202 - ps_3.getValue()
# phi_r1 = 3.03552063 - ps_2.getValue()
# phi_r4 = 0.10607202 - ps_4.getValue()

# _, _, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
# _, _, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

# _, theta_b, fai_init = imu.getRollPitchYaw()
# theta_ll = math.pi / 2 - phi0_l + theta_b
# theta_lr = math.pi / 2 - phi0_r + theta_b

# theta_wl_init = ps_5.getValue()
# theta_wr_init = ps_6.getValue()
# s_leg_init = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr))

# theta_l1.last_diff = phi_l1
# theta_l4.last_diff = phi_l4
# theta_r1.last_diff = phi_r1
# theta_r4.last_diff = phi_r4
# Theta_wl.last_diff = theta_wl_init
# Theta_wr.last_diff = theta_wr_init

# # 解锁后立即进入闭环
# robot_node.resetPhysics()
# locked_field.setSFBool(False)
# start_time = robot.getTime()
# counter = 0
# print_interval = max(1, int(100 / timestep))
# print('初值同步完成，10 状态 LQR 开始工作。', flush=True)

# # Main loop:
# while robot.step(timestep) != -1:
#     current_time = robot.getTime() - start_time

#     # 机器人基本状态获取：当前模型用 pitch 和绕 y 轴角速度
#     roll, pitch, fai = imu.getRollPitchYaw()
#     theta_b = pitch
#     fai = math.atan2(math.sin(fai - fai_init), math.cos(fai - fai_init))
#     dot_roll, dot_theta_b, dot_fai = gyro.getValues()

#     # 五连杆关节角
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = math.pi / 2 - phi0_l + theta_b
#     theta_lr = math.pi / 2 - phi0_r + theta_b

#     # 当前模型的轮编码器直接读取，不加负号
#     theta_wl = ps_5.getValue()
#     theta_wr = ps_6.getValue()
#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     omega1l = theta_l1.Diff(phi_l1)
#     omega4l = theta_l4.Diff(phi_l4)
#     omega1r = theta_r1.Diff(phi_r1)
#     omega4r = theta_r4.Diff(phi_r4)

#     # leg.spd 第二个返回值是 -dot(phi0)
#     L0_speedl, phi0_speedl = leg.spd(omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4)
#     L0_speedr, phi0_speedr = leg.spd(omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4)
#     dot_theta_ll = phi0_speedl + dot_theta_b
#     dot_theta_lr = phi0_speedr + dot_theta_b

#     # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
#     s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
#     s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
#     # s = s_wheel + s_leg

#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_leg = 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll)
#                       + L0_speedl * math.sin(theta_ll)
#                       + L0_r * dot_theta_lr * math.cos(theta_lr)
#                       + L0_speedr * math.sin(theta_lr))
#     dot_s_b = dot_s_wheel + dot_s_leg

#     # # 高速时退出位置反馈，低速时积分位移进行位置保持
#     if abs(dot_s_b) < 0.1:
#         s_local += dot_s_b * d_t
#     else:
#         s_local = 0.0

#     s = s_local

#     # 状态方程的十个状态量
#     real_state = np.matrix([[s],
#                             [dot_s_b],
#                             [fai],
#                             [dot_fai],
#                             [theta_ll],
#                             [dot_theta_ll],
#                             [theta_lr],
#                             [dot_theta_lr],
#                             [theta_b],
#                             [dot_theta_b]])

#     # 根据时间设置腿长目标，首尾速度为零
#     leg_ratio = float(np.clip((current_time - leg_hold_time) / leg_change_time, 0, 1))
#     leg_ratio = leg_ratio * leg_ratio * (3 - 2 * leg_ratio)
#     target_L0_l = start_L0 + leg_ratio * (final_L0 - start_L0)
#     target_L0_r = target_L0_l

#     # 根据时间规定目标状态，直接修改各段矩阵中的数值即可
#     # 顺序：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll,
#     #       theta_lr, dot_theta_lr, theta_b, dot_theta_b
#     # 腿长由上面的 target_L0_l / target_L0_r 单独控制
#     if 0 <= current_time <= 2:
#         # 0～2 秒：保持原位、原朝向，腿和机体保持直立
#         expect_state = np.matrix([[0],               # 位移，m
#                                   [0],               # 速度，m/s
#                                   [0],               # 偏航角，rad
#                                   [0],               # 偏航角速度，rad/s
#                                   [0],               # 左腿摆角，rad
#                                   [0],               # 左腿摆角速度，rad/s
#                                   [0],               # 右腿摆角，rad
#                                   [0],               # 右腿摆角速度，rad/s
#                                   [0],               # 机体俯仰角，rad
#                                   [0]])              # 机体俯仰角速度，rad/s

#     elif 2 < current_time <= 2.5:
#         # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
#         expect_state = np.matrix([[0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     else:
#         # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
#         expect_state = np.matrix([[0],
#                                   [0.5],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     K = (1 - leg_ratio) * K_start + leg_ratio * K_final

#     # 解算力矩
#     U = K * (expect_state - real_state)
#     T_l = float(U[0, 0])       # 左轮力矩
#     T_r = float(U[1, 0])       # 右轮力矩
#     T_pl = float(U[2, 0])      # 左腿摆力矩
#     T_pr = float(U[3, 0])      # 右腿摆力矩

#     # VMC 关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长 PD：保持已经验证的速度阻尼写法
#     dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl
#     dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr
#     F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
#     F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

#     T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
#     T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

#     T1_l = float(np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque))  # 左前
#     T1_r = float(np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque))  # 左后
#     T2_l = float(np.clip(T_JOINT_R[0, 0], -max_hip_torque, max_hip_torque))  # 右前
#     T2_r = float(np.clip(T_JOINT_R[1, 0], -max_hip_torque, max_hip_torque))  # 右后

#     # 最终赋值：当前模型轮力矩与 LQR 输出同号
#     motor_1.setTorque(T1_l)
#     motor_3.setTorque(T1_r)
#     motor_2.setTorque(T2_l)
#     motor_4.setTorque(T2_r)
#     motor_5.setTorque(float(np.clip(T_l, -max_wheel_torque, max_wheel_torque)))
#     motor_6.setTorque(float(np.clip(T_r, -max_wheel_torque, max_wheel_torque)))

#     # 每 100 ms 打印一次
#     counter += 1
#     if counter >= print_interval:
#         counter = 0
#         print(f"t={current_time:.2f}s, target_L0={target_L0_l:.4f} m")
#         print(
#             f"目标：s={float(expect_state[0, 0]):+.3f} m, "
#             f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
#             f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
#         )
#         print(
#             f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, "
#             f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
#             f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
#         )
#         print(
#             f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
#         )
#         print(f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m")
#         print(
#             f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
#             f"髋命令：M1={T1_l:+.2f}, M3={T1_r:+.2f}, M2={T2_l:+.2f}, M4={T2_r:+.2f} N*m"
#         )
#         print("--------------------------------")





















# from IMU import IMU_yaw
# import math
# import lagrange
# import leg
# from controller import Motor, PositionSensor, Gyro, Accelerometer, Robot, InertialUnit
# import mymath
# import numpy as np

# robot = Robot()
# timestep = int(robot.getBasicTimeStep())

# gyro = Gyro("gyro")
# imu = InertialUnit("imu")
# accelerometer = Accelerometer("accelerometer")
# robot_yaw = IMU_yaw()

# imu.enable(timestep)
# gyro.enable(timestep)
# accelerometer.enable(timestep)


# # 髋电机
# motor_1 = Motor("Left_Front_Motor")  # 左腿第一个主动关节
# motor_3 = Motor("Left_Back_Motor")
# motor_2 = Motor("Right_Front_Motor")  # 右腿第一个主动关节
# motor_4 = Motor("Right_Back_Motor")

# # 足电机
# motor_5 = Motor("Left_Wheel")  # 安装反的，设置力矩要加负号
# motor_6 = Motor("Right_Wheel")  # 安装反的，设置力矩要加负号

# # 髋角度
# ps_1 = PositionSensor("Left_Front_Motor_Sensor")
# ps_3 = PositionSensor("Left_Back_Motor_Sensor")
# ps_2 = PositionSensor("Right_Front_Motor_Sensor")
# ps_4 = PositionSensor("Right_Back_Motor_Sensor")

# # 足角度
# ps_5 = PositionSensor("Left_Wheel_Sensor")  # 安装反的，读取数据要加负号
# ps_6 = PositionSensor("Right_Wheel_Sensor")  # 安装反的，读取数据要加负号


# # 初始化电机
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setPosition(0.0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, accelerometer, imu]:
#     sensor.enable(timestep)


# # 差分初始化
# d_t = timestep / 1000
# Theta_b = mymath.Discreteness(d_t)

# diff_yaw = mymath.Discreteness(d_t)
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# Roll = mymath.Discreteness(d_t)

# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# d_Ll = mymath.Discreteness(d_t)
# d_Lr = mymath.Discreteness(d_t)

# Ll = mymath.Discreteness(d_t)

# Ll.last_diff = 0.16000280

# theta_l1.last_diff = 3.03552063
# theta_r1.last_diff = 3.03552063
# theta_l4.last_diff = 0.10607202
# theta_r4.last_diff = 0.10607202

# target_L0_l = 0.38
# target_L0_r = 0.38

# counter = 0
# print_interval = max(1, int(100 / timestep))  # 每100ms打印一次


# # PID初始化
# F0_control_l = mymath.PID_control(400, 0, 4000, target_L0_l)
# F0_control_r = mymath.PID_control(400, 0, 4000, target_L0_r)

# # 轮子半径
# r = 0.06
# s = 0.0

# # 左腿两个电机采用位置控制
# motor_1.setVelocity(0.2)
# motor_3.setVelocity(0.2)
# motor_2.setVelocity(0.2)
# motor_4.setVelocity(0.2)
# # 绝对目标位置都是1 rad
# motor_1.setPosition(0.3)
# motor_3.setPosition(0.3)
# motor_2.setPosition(0.3)
# motor_4.setPosition(0.3)

# current_time = 0
# flag = 0
# expect_state = np.zeros((10, 1))

# while robot.step(timestep) != -1:
#     current_time = current_time + d_t

#     # 获取姿态角
#     roll, pitch, yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(yaw)

#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()
#     ax, ay, az = accelerometer.getValues()

#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()

#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = 1.570796 - phi0_l + theta_b
#     theta_lr = 1.570796 - phi0_r + theta_b

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()

#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     # 四个主动关节角速度，单位 rad/s
#     dot_phi_l1 = theta_l1.Diff(phi_l1)
#     dot_phi_l4 = theta_l4.Diff(phi_l4)
#     dot_phi_r1 = theta_r1.Diff(phi_r1)
#     dot_phi_r4 = theta_r4.Diff(phi_r4)

#     # leg.spd()第二个返回值是 -dot_phi0
#     # 也就是虚拟腿相对机身的摆动角速度
#     dot_L0_l, dot_Phi0_l_negative = leg.spd(
#         dot_phi_l1,
#         dot_phi_l4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_l1,
#         phi_l4,
#     )

#     dot_L0_r, dot_Phi0_r_negative = leg.spd(
#         dot_phi_r1,
#         dot_phi_r4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_r1,
#         phi_r4,
#     )

#     # 腿长伸缩加速度
#     ddot_L0_l = d_Ll.Diff(dot_L0_l)
#     ddot_L0_r = d_Lr.Diff(dot_L0_r)

#     dot_theta_ll = dot_Phi0_l_negative + dot_theta_b
#     dot_theta_lr = dot_Phi0_r_negative + dot_theta_b

#     # 位移与速度
#     # s = r * (theta_wl + theta_wr) / 2
#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_b = (
#         dot_s_wheel
#         + 0.5
#         * (
#             L0_l * dot_theta_ll * math.cos(theta_ll)
#             + L0_r * dot_theta_lr * math.cos(theta_lr)
#         )
#         + 0.5 * (dot_L0_l * math.sin(theta_ll) + dot_L0_r * math.sin(theta_lr))
#     )
#     s += dot_s_b * d_t

#     # 当前的状态
#     # current_state = np.matrix(
#     #     [
#     #         [s],
#     #         [dot_s_b],
#     #         [yaw],
#     #         [dot_yaw],
#     #         [theta_ll],
#     #         [dot_theta_ll],
#     #         [theta_lr],
#     #         [dot_theta_lr],
#     #         [theta_b],
#     #         [dot_theta_b],
#     #     ]
#     # )
#     current_state = np.matrix(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ]
#     )

#     # 计算虚拟腿力到两个髋电机力矩的转换矩阵
#     if 0 < current_time <= 3:
#         flag = 0
#     elif 3 < current_time < 1000:
#         expect_state = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
#         flag = 1
#     else:
#         break
#     K = lagrange.K(
#         target_L0_l, target_L0_r, 50, 1, 500, 1, 500, 1, 500, 1, 5000, 1, 1, 1, 1, 1
#     )

#     U = K * (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     T_wl = 0
#     T_wr = 0

#     # VMC关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长PID
#     pid_force_l = F0_control_l.position_pid(L0_l)
#     pid_force_r = F0_control_r.position_pid(L0_r)

#     F_l = -pid_force_l
#     F_r = -pid_force_r
#     # 第二项为虚拟腿摆动力矩，本次只测腿长，所以设为0
#     T_JOINT_L = JRM_L * np.matrix(
#         [
#             [F_l],
#             # [0],
#             # [T_ll],
#             [0],
#         ]
#     )

#     T_JOINT_R = JRM_R * np.matrix(
#         [
#             [F_r],
#             # [0],
#             # [T_lr],
#             [0],
#         ]
#     )

#     T1_l = float(T_JOINT_L[0, 0])
#     T1_r = float(T_JOINT_L[1, 0])
#     T2_l = float(T_JOINT_R[0, 0])
#     T2_r = float(T_JOINT_R[1, 0])

#     motor_1.setTorque(float(np.clip(T1_l, -25.0, 25.0)))
#     motor_3.setTorque(float(np.clip(T1_r, -25.0, 25.0)))
#     motor_2.setTorque(float(np.clip(T2_l, -25.0, 25.0)))
#     motor_4.setTorque(float(np.clip(T2_r, -25.0, 25.0)))
#     motor_5.setTorque(float(np.clip(-T_wl, -10.0, 10.0)))
#     motor_6.setTorque(float(np.clip(-T_wr, -10.0, 10.0)))

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         print(
#             f"theta_b={math.degrees(theta_b):+.2f}°, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f}°/s"
#         )

#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"theta_ll={math.degrees(theta_ll):+.2f}°, "
#             f"phi_l1={phi_l1:+.2f}rad, "
#             f"phi_l4={phi_l4:+.2f}rad, "
#             f"ps_1={ps_1.getValue():+.2f}rad, "
#             f"ps_4={ps_4.getValue():+.2f}rad, "
#             f"dot_theta_ll={math.degrees(dot_theta_ll):+.2f}°/s,"
#             f"T1_l={T1_l}N, "
#             f"T1_r={T1_r}N"
#         )

#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"theta_lr={math.degrees(theta_lr):+.2f}°, "
#             f"dot_theta_lr={math.degrees(dot_theta_lr):+.2f}°/s"
#             f"T2_l={T2_l}N, "
#             f"T2_r={T2_r}N"
#         )

#         print(
#             f"phi0_l={math.degrees(phi0_l):+.4f} °, "
#             f"phi0_r={math.degrees(phi0_r):+.4f} °, "
#             f"flag={flag} , "
#             f"pid_force_l={pid_force_l:+.2f}N, "
#             f"pid_force_r={pid_force_r:+.2f}N"
#         )

#         print("--------------------------------")


















# """Webots 五连杆腿摆 LQR 极性测试。

# 测试流程：
# 1. 0～3 秒：四个髋电机使用位置控制，让左右虚拟腿同时偏转。
# 2. 3 秒以后：髋电机只切换一次到力矩控制，由 LQR 使腿摆角回到 0。
# 3. 同时以小力矩缓慢启用两个轮子，用于验证轮子输出极性。

# 运行本测试前，请在 Webots 中把 Robot 根节点设置为 locked TRUE。
# """

# import math

# import numpy as np
# from controller import Robot

# from IMU import IMU_yaw
# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# PERTURB_TIME = 3.0                 # 前 3 秒制造腿摆偏角
# PERTURB_MOTOR_POSITION = 0.50      # rad，约 17.2 度
# POSITION_SPEED = 0.20              # rad/s

# # 极性测试必须从小力矩开始，确认方向正确后再逐步增大。
# HIP_TORQUE_LIMIT = 0.30            # N*m

# # 当前测试没有启用腿长 PID，腿长仍接近初始的 0.16 m。
# LQR_MODEL_LEG_LENGTH = 0.16000280  # m

# # 已确认髋关节腿摆极性后，启用轮子做第二阶段测试。
# ENABLE_WHEEL_TORQUE = True
# WHEEL_TORQUE_LIMIT = 0.10          # N*m
# WHEEL_RAMP_TIME = 1.0              # s，轮力矩在 1 秒内由 0 缓慢增加


# robot = Robot()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)

# robot_yaw = IMU_yaw()


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)

# theta_l1_diff.last_diff = 3.03552063
# theta_r1_diff.last_diff = 3.03552063
# theta_l4_diff.last_diff = 0.10607202
# theta_r4_diff.last_diff = 0.10607202


# # ----------------------- 0～3 秒位置控制 -----------------------
# # 这里只发送位置命令。此阶段不能再调用四个髋电机的 setTorque()。
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(POSITION_SPEED)

# # 左右腿同时向同一腿摆方向偏转。
# motor_1.setPosition(PERTURB_MOTOR_POSITION)
# motor_3.setPosition(PERTURB_MOTOR_POSITION)
# motor_2.setPosition(PERTURB_MOTOR_POSITION)
# motor_4.setPosition(PERTURB_MOTOR_POSITION)

# # 轮子一直使用力矩模式。
# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# # K 只计算一次，避免每个仿真周期重复求解 Riccati 方程。
# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print("控制器启动：前 3 秒位置控制，3 秒后切换到腿摆 LQR。", flush=True)


# while robot.step(timestep) != -1:
#     current_time = robot.getTime()

#     # IMU：模型 x 向前、z 向上时，pitch/绕 y 轴角速度对应机体俯仰。
#     roll, pitch, raw_yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(raw_yaw)
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 主动关节角转换到五连杆运动学角度。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     # leg.spd() 的第二个输出是 -dot(phi0)。
#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 保留你当前模型的轮子坐标约定：控制坐标 = -PositionSensor 原始值。
#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 本测试假设其余 6 个状态为 0，只把两条腿的摆角和角速度送入 LQR。
#     current_state = np.array(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ],
#         dtype=float,
#     )

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 把虚拟腿摆力矩映射到四个髋关节电机。
#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     # Webots 原始轮电机命令。模型轮力矩与电机命令之间保留一个负号。
#     wheel_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if current_time < PERTURB_TIME:
#         phase = "POSITION"

#         # 关键：这里不能调用 motor_1～motor_4.setTorque()。
#         # 四个髋电机会继续完成循环外设置的 0.30 rad 位置目标。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     else:
#         phase = "LQR"

#         # 到达 3 秒时只执行一次位置模式 -> 力矩模式切换。
#         if not released:
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             released = True
#             print("3 秒到达：已释放位置控制，开始 LQR 回中。", flush=True)

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)

#         if ENABLE_WHEEL_TORQUE:
#             # 切换后缓慢投入轮力矩，防止瞬间冲击。
#             wheel_ramp = float(
#                 np.clip((current_time - PERTURB_TIME) / WHEEL_RAMP_TIME, 0.0, 1.0)
#             )
#             cmd_wl = float(
#                 np.clip(T_wl * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             motor_5.setTorque(cmd_wl)
#             motor_6.setTorque(cmd_wr)
#         else:
#             motor_5.setTorque(0.0)
#             motor_6.setTorque(0.0)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "LQR":
#             left_trend = "回中" if theta_ll * dot_theta_ll < 0.0 else "离开/暂未运动"
#             right_trend = "回中" if theta_lr * dot_theta_lr < 0.0 else "离开/暂未运动"

#             if abs(T_wl) < 1e-4:
#                 left_wheel_polarity = "力矩接近0"
#             elif T_wl * dot_theta_wl > 0.0:
#                 left_wheel_polarity = "极性一致"
#             else:
#                 left_wheel_polarity = "反向/暂未转动"

#             if abs(T_wr) < 1e-4:
#                 right_wheel_polarity = "力矩接近0"
#             elif T_wr * dot_theta_wr > 0.0:
#                 right_wheel_polarity = "极性一致"
#             else:
#                 right_wheel_polarity = "反向/暂未转动"
#         else:
#             left_trend = "制造偏角"
#             right_trend = "制造偏角"
#             left_wheel_polarity = "尚未启用"
#             right_wheel_polarity = "尚未启用"

#         print(
#             f"t={current_time:5.2f}s  phase={phase}  "
#             f"theta_b={math.degrees(theta_b):+.2f} deg"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"Phi0={math.degrees(phi0_l):+.2f} deg, "
#             f"theta={theta_ll:+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s, "
#             f"phi_l1={phi_l1:+.2f} deg, "
#             f"phi_l4={phi_l4:+.2f} deg, "
#             f"判断={left_trend}"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"Phi0={math.degrees(phi0_r):+.2f} deg, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s, "
#             f"phi_r1={math.degrees(phi_r1):+.2f} deg, "
#             f"phi_r4={math.degrees(phi_r4):+.2f} deg, "
#             f"判断={right_trend}"
#         )
#         print(
#             f"LQR：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"髋电机命令：M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"左轮：模型速度={dot_theta_wl:+.4f} rad/s, "
#             f"电机命令={cmd_wl:+.4f} N*m, 判断={left_wheel_polarity}"
#         )
#         print(
#             f"右轮：模型速度={dot_theta_wr:+.4f} rad/s, "
#             f"电机命令={cmd_wr:+.4f} N*m, 判断={right_wheel_polarity}, "
#             f"ramp={wheel_ramp:.2f}"
#         )
#         print("--------------------------------")










# """Webots 机体俯仰 LQR 极性测试。

# 状态向量中只有 theta_b、dot_theta_b 使用真实测量值，其余 8 个状态全部为 0。

# 通过 TEST_MODE 分别测试：
# - "LEG_ONLY"：只施加虚拟腿摆力矩，两个轮子的力矩严格为 0；
# - "WHEEL_ONLY"：只施加轮力矩，四个髋关节电机的力矩严格为 0；
# - "BOTH"：腿摆与轮子同时工作，仅用于前两项通过后的组合验证。

# 流程：
# 1. 0～3 秒：Supervisor 固定 Robot 根节点，并将机体设置为 5 度初始倾角；
#    四个髋关节保持零位，轮力矩为 0。
# 2. 3 秒以后：解锁 Robot，髋关节切换到力矩模式，由机体状态产生的
#    LQR 轮力矩和虚拟腿摆力矩开始工作。

# WBT 使用要求：
# - Robot 节点 supervisor TRUE
# - 轮子接触地面
# - Robot 的 locked 初始值 TRUE/FALSE 均可，代码会自动控制
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次运行选 "LEG_ONLY"，复位仿真后改成 "WHEEL_ONLY" 再运行一次。
# TEST_MODE = "BOTH"
# VALID_TEST_MODES = ("LEG_ONLY", "WHEEL_ONLY", "BOTH")

# if TEST_MODE not in VALID_TEST_MODES:
#     raise ValueError(f"TEST_MODE 必须是 {VALID_TEST_MODES} 之一")

# RELEASE_TIME = 3.0
# INITIAL_BODY_PITCH = math.radians(-5.0)

# LQR_MODEL_LEG_LENGTH = 0.16000280

# HIP_POSITION_SPEED = 0.20
# HIP_TORQUE_LIMIT = 0.30
# WHEEL_TORQUE_LIMIT = 0.10
# TORQUE_RAMP_TIME = 0.50

# # 倾角超过该值就关闭全部力矩并重新固定机体，避免极性错误时摔飞。
# SAFETY_PITCH = math.radians(25.0)


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请把 Robot 的 supervisor 设置为 TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")

# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先固定机体，再设置绕 y 轴的初始俯仰角。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # ----------------------- 0～3 秒保持腿零位 -----------------------
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(HIP_POSITION_SPEED)
#     hip_motor.setPosition(0.0)

# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- 轮速差分 ---------------------------
# # 保留你当前使用的轮子反馈坐标：theta_w = -PositionSensor。
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print(
#     f"控制器启动：TEST_MODE={TEST_MODE}，机体固定在 5 度，"
#     "3 秒后释放并启动仅机体状态的 LQR。",
#     flush=True,
# )

# controller_start_time = robot.getTime()


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 运动学只用于把 LQR 的虚拟腿摆力矩映射到四个髋电机。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 10 维状态顺序：
#     # s, ds, yaw, dyaw, theta_ll, dtheta_ll,
#     # theta_lr, dtheta_lr, theta_b, dtheta_b
#     #
#     # 本测试只有最后两个机体状态不是 0。
#     current_state = np.zeros((10, 1))
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     # 只输入虚拟腿摆力矩，轴向力暂时为 0。
#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     torque_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if elapsed_time < RELEASE_TIME:
#         phase = "HOLD"

#         # 髋电机继续执行零位位置控制；此处不能调用髋电机 setTorque()。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     elif not safety_stopped:
#         phase = "BODY_LQR"

#         if not released:
#             # 髋关节切换到力矩模式。
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             # 清除固定阶段的物理速度后释放机体。
#             robot_node.resetPhysics()
#             locked_field.setSFBool(False)
#             released = True

#             print("3 秒到达：机体已释放，启动机体俯仰 LQR。", flush=True)

#         torque_ramp = float(
#             np.clip((elapsed_time - RELEASE_TIME) / TORQUE_RAMP_TIME, 0.0, 1.0)
#         )

#         # 根据 TEST_MODE 只允许被测执行器产生实际力矩。
#         if TEST_MODE in ("LEG_ONLY", "BOTH"):
#             cmd_m1 *= torque_ramp
#             cmd_m3 *= torque_ramp
#             cmd_m2 *= torque_ramp
#             cmd_m4 *= torque_ramp
#         else:
#             # WHEEL_ONLY：髋关节已经进入力矩模式，但实际命令严格为 0。
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0

#         if TEST_MODE in ("WHEEL_ONLY", "BOTH"):
#             # 已按你的实测结果修正：LQR轮力矩和Webots电机命令同号。
#             cmd_wl = float(
#                 np.clip(T_wl * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#         else:
#             # LEG_ONLY：轮子保持力矩模式，但实际命令严格为 0。
#             cmd_wl = 0.0
#             cmd_wr = 0.0

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#         if abs(theta_b) > SAFETY_PITCH:
#             for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#                 motor.setTorque(0.0)

#             locked_field.setSFBool(True)
#             safety_stopped = True
#             phase = "SAFETY_STOP"
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0
#             cmd_wl = 0.0
#             cmd_wr = 0.0
#             print("安全停止：机体倾角超过 25 度，请检查极性。", flush=True)

#     else:
#         phase = "SAFETY_STOP"

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "BODY_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 body_trend = "接近中心"
#             elif theta_b * dot_theta_b < 0.0:
#                 body_trend = "正在回正"
#             else:
#                 body_trend = "离开中心/暂未运动"
#         elif phase == "HOLD":
#             body_trend = "保持初始倾角"
#         else:
#             body_trend = "已停止"

#         print(
#             f"t={elapsed_time:5.2f}s, mode={TEST_MODE}, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"判断={body_trend}"
#         )
#         print(
#             "输入状态："
#             f"theta_b={current_state[8, 0]:+.5f} rad, "
#             f"dot_theta_b={current_state[9, 0]:+.5f} rad/s，"
#             "其余状态全部为0"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"实际命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f}, "
#             f"M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"轮速：左={dot_theta_wl:+.3f}, 右={dot_theta_wr:+.3f} rad/s, "
#             f"ramp={torque_ramp:.2f}"
#         )
#         print("--------------------------------")


# """Webots 轮腿机器人 6 状态初步平衡测试。

# LQR 中仅使用以下 6 个真实状态：
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# s、dot_s、yaw、dot_yaw 全部置 0。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# # TARGET_LEG_LENGTH_L = 0.16000280
# # TARGET_LEG_LENGTH_R = 0.16000280
# TARGET_LEG_LENGTH_L = 0.2000280
# TARGET_LEG_LENGTH_R = 0.2000280

# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# WHEEL_TORQUE_LIMIT = 10.10    # N*m，第一次平衡测试从小力矩开始

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 6 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 10维向量中只有腿摆4状态和机体2状态是真实值。
#     # s、dot_s、yaw、dot_yaw 全部为0。
#     current_state = np.zeros((10, 1))
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "SIX_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [theta_b, dot_theta_b, theta_ll, theta_lr, L0_l, L0_r]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "SIX_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "6状态输入："
#             f"[{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# TARGET_LEG_LENGTH_L = 0.16000280
# TARGET_LEG_LENGTH_R = 0.16000280
# # TARGET_LEG_LENGTH_L = 0.20
# # TARGET_LEG_LENGTH_R = 0.20


# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     500, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡与变腿长测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 8 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到 0.20 m，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 8 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.220
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")


# """完整 10 状态轮腿平衡与变腿长测试。

# LQR 中使用以下 10 个真实状态：
#     s, dot_s, yaw, dot_yaw,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# 启动时的实际朝向作为 yaw=0，原来已经验证的腿长 PD、VMC 映射
# 以及轮子和髋关节输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 10 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到设定长度，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 10 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 腿长到达终点后再原地转向 +pi/2。yaw 目标也使用 smoothstep，避免
# # 直接给 90 度阶跃造成左右轮差动力矩瞬间饱和。
# FINAL_YAW = math.pi / 2.0
# YAW_RAMP_TIME = 4.0

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0
# YAW_HOLD_TIME = LEG_LENGTH_HOLD_TIME + LEG_LENGTH_RAMP_TIME

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 完整状态首次联调仍保留轮力矩限幅，避免偏航或姿态误差导致瞬时冲击。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_YAW = math.radians(120.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.240
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     1000, 5,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、偏航角、腿摆角、轮角以及世界位置全部作为零点。
# _, theta_b_init, yaw_init = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与完整 10 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 5 秒前保持原朝向，5～9 秒平滑转到 +pi/2，之后保持 90 度。
#     yaw_linear_ratio = float(np.clip(
#         (elapsed_time - YAW_HOLD_TIME) / YAW_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     yaw_ramp_ratio = (
#         yaw_linear_ratio * yaw_linear_ratio * (3.0 - 2.0 * yaw_linear_ratio)
#     )
#     target_yaw = FINAL_YAW * yaw_ramp_ratio

#     if elapsed_time < YAW_HOLD_TIME:
#         yaw_phase = "HOLD"
#     elif yaw_linear_ratio < 1.0:
#         yaw_phase = "TURN"
#     else:
#         yaw_phase = "FINAL"

#     expect_state[2, 0] = target_yaw

#     # 机体姿态。yaw 使用启动时朝向作为 0，并包角到 [-pi, pi]。
#     roll, theta_b, raw_yaw = imu.getRollPitchYaw()
#     yaw = math.atan2(
#         math.sin(raw_yaw - yaw_init),
#         math.cos(raw_yaw - yaw_init),
#     )
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 完整 10 状态，顺序必须与 lagrange.py 建模顺序完全一致。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[2, 0] = yaw
#     current_state[3, 0] = dot_yaw
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "TEN_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world, yaw, dot_yaw,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or abs(yaw) > SAFETY_YAW
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if abs(yaw) > SAFETY_YAW:
#             stop_reasons.append("偏航角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "TEN_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"偏航计划：{yaw_phase}, ramp={yaw_ramp_ratio:.3f}, "
#             f"target={math.degrees(target_yaw):+.2f} deg"
#         )
#         print(
#             f"偏航反馈：yaw={math.degrees(yaw):+.2f} deg, "
#             f"error={math.degrees(target_yaw - yaw):+.2f} deg, "
#             f"dot_yaw={math.degrees(dot_yaw):+.2f} deg/s"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "10状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{yaw:+.4f}, {dot_yaw:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"轮力矩分量：平均={(T_wl + T_wr) / 2.0:+.4f}, "
#             f"差动={(T_wr - T_wl) / 2.0:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")



























# # -*- coding: utf-8 -*-
# # my_controller.py：10 状态 LQR + 腿长 PD

# import math
# import numpy as np

# import lagrange
# import leg
# import mymath
# from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit

# robot = Supervisor()  # Webots 的 Robot 节点需设置 supervisor TRUE
# timestep = int(robot.getBasicTimeStep())
# d_t = timestep / 1000

# # 初始机体倾角
# initial_body_pitch = math.radians(0.0)
# robot_node = robot.getSelf()
# locked_field = robot_node.getField('locked')
# locked_field.setSFBool(True)
# robot_node.getField('rotation').setSFRotation([0, 1, 0, initial_body_pitch])
# robot_node.resetPhysics()

# # 足电机
# motor_5 = Motor('Left_Wheel')
# motor_6 = Motor('Right_Wheel')

# # 髋电机
# motor_3 = Motor('Left_Front_Motor')   # 左后
# motor_1 = Motor('Left_Back_Motor')  # 左前
# motor_4 = Motor('Right_Front_Motor')  # 右后
# motor_2 = Motor('Right_Back_Motor') # 右前


# # 足角度
# ps_5 = PositionSensor('Left_Wheel_Sensor')
# ps_6 = PositionSensor('Right_Wheel_Sensor')

# # 髋角度
# ps_1 = PositionSensor('Left_Back_Motor_Sensor')
# ps_3 = PositionSensor('Left_Front_Motor_Sensor')
# ps_2 = PositionSensor('Right_Back_Motor_Sensor')
# ps_4 = PositionSensor('Right_Front_Motor_Sensor')

# gyro = Gyro('gyro')
# imu = InertialUnit('imu')

# # 初始化电机：直接力矩控制
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float('inf'))
#     motor.setVelocity(0)
#     motor.setTorque(0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, imu]:
#     sensor.enable(timestep)

# # 差分初始化
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# # 腿长目标：先保持 2 秒，再用 0.5 秒从 0.16 m 平滑变到 0.22 m
# start_L0 = 0.16000280
# final_L0 = 0.22
# leg_hold_time = 2.0
# leg_change_time = 0.5

# # 腿长 PD 参数与支撑力前馈
# leg_kp = 400.0
# leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
# gravity_force_l = 22.0        # N
# gravity_force_r = 22.0        # N

# # 输出限幅
# max_leg_force = 40.0         # N
# max_hip_torque = 10.0        # N*m
# max_wheel_torque = 0.50      # N*m
# r = 0.05995                  # 轮半径，m

# # 两组 K 只在启动时计算，之后随腿长目标插值
# K_start = lagrange.K(start_L0, start_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)
# K_final = lagrange.K(final_L0, final_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)

# s_local = 0.0
# # 采集一拍真实初值，避免差分器第一拍产生虚假速度
# if robot.step(timestep) == -1:
#     raise RuntimeError('仿真在控制器初始化时结束')

# phi_l1 = 3.03552063 - ps_1.getValue()
# phi_l4 = 0.10607202 - ps_3.getValue()
# phi_r1 = 3.03552063 - ps_2.getValue()
# phi_r4 = 0.10607202 - ps_4.getValue()

# _, _, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
# _, _, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

# _, theta_b, fai_init = imu.getRollPitchYaw()
# theta_ll = math.pi / 2 - phi0_l + theta_b
# theta_lr = math.pi / 2 - phi0_r + theta_b

# theta_wl_init = ps_5.getValue()
# theta_wr_init = ps_6.getValue()
# s_leg_init = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr))

# theta_l1.last_diff = phi_l1
# theta_l4.last_diff = phi_l4
# theta_r1.last_diff = phi_r1
# theta_r4.last_diff = phi_r4
# Theta_wl.last_diff = theta_wl_init
# Theta_wr.last_diff = theta_wr_init

# # 解锁后立即进入闭环
# robot_node.resetPhysics()
# locked_field.setSFBool(False)
# start_time = robot.getTime()
# counter = 0
# print_interval = max(1, int(100 / timestep))
# print('初值同步完成，10 状态 LQR 开始工作。', flush=True)

# # Main loop:
# while robot.step(timestep) != -1:
#     current_time = robot.getTime() - start_time

#     # 机器人基本状态获取：当前模型用 pitch 和绕 y 轴角速度
#     roll, pitch, fai = imu.getRollPitchYaw()
#     theta_b = pitch
#     fai = math.atan2(math.sin(fai - fai_init), math.cos(fai - fai_init))
#     dot_roll, dot_theta_b, dot_fai = gyro.getValues()

#     # 五连杆关节角
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = math.pi / 2 - phi0_l + theta_b
#     theta_lr = math.pi / 2 - phi0_r + theta_b

#     # 当前模型的轮编码器直接读取，不加负号
#     theta_wl = ps_5.getValue()
#     theta_wr = ps_6.getValue()
#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     omega1l = theta_l1.Diff(phi_l1)
#     omega4l = theta_l4.Diff(phi_l4)
#     omega1r = theta_r1.Diff(phi_r1)
#     omega4r = theta_r4.Diff(phi_r4)

#     # leg.spd 第二个返回值是 -dot(phi0)
#     L0_speedl, phi0_speedl = leg.spd(omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4)
#     L0_speedr, phi0_speedr = leg.spd(omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4)
#     dot_theta_ll = phi0_speedl + dot_theta_b
#     dot_theta_lr = phi0_speedr + dot_theta_b

#     # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
#     s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
#     s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
#     # s = s_wheel + s_leg

#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_leg = 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll)
#                       + L0_speedl * math.sin(theta_ll)
#                       + L0_r * dot_theta_lr * math.cos(theta_lr)
#                       + L0_speedr * math.sin(theta_lr))
#     dot_s_b = dot_s_wheel + dot_s_leg

#     # # 高速时退出位置反馈，低速时积分位移进行位置保持
#     if abs(dot_s_b) < 0.1:
#         s_local += dot_s_b * d_t
#     else:
#         s_local = 0.0

#     s = s_local

#     # 状态方程的十个状态量
#     real_state = np.matrix([[s],
#                             [dot_s_b],
#                             [fai],
#                             [dot_fai],
#                             [theta_ll],
#                             [dot_theta_ll],
#                             [theta_lr],
#                             [dot_theta_lr],
#                             [theta_b],
#                             [dot_theta_b]])

#     # 根据时间设置腿长目标，首尾速度为零
#     leg_ratio = float(np.clip((current_time - leg_hold_time) / leg_change_time, 0, 1))
#     leg_ratio = leg_ratio * leg_ratio * (3 - 2 * leg_ratio)
#     target_L0_l = start_L0 + leg_ratio * (final_L0 - start_L0)
#     target_L0_r = target_L0_l

#     # 根据时间规定目标状态，直接修改各段矩阵中的数值即可
#     # 顺序：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll,
#     #       theta_lr, dot_theta_lr, theta_b, dot_theta_b
#     # 腿长由上面的 target_L0_l / target_L0_r 单独控制
#     if 0 <= current_time <= 2:
#         # 0～2 秒：保持原位、原朝向，腿和机体保持直立
#         expect_state = np.matrix([[0],               # 位移，m
#                                   [0],               # 速度，m/s
#                                   [0],               # 偏航角，rad
#                                   [0],               # 偏航角速度，rad/s
#                                   [0],               # 左腿摆角，rad
#                                   [0],               # 左腿摆角速度，rad/s
#                                   [0],               # 右腿摆角，rad
#                                   [0],               # 右腿摆角速度，rad/s
#                                   [0],               # 机体俯仰角，rad
#                                   [0]])              # 机体俯仰角速度，rad/s

#     elif 2 < current_time <= 2.5:
#         # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
#         expect_state = np.matrix([[0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     else:
#         # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
#         expect_state = np.matrix([[0],
#                                   [0.5],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     K = (1 - leg_ratio) * K_start + leg_ratio * K_final

#     # 解算力矩
#     U = K * (expect_state - real_state)
#     T_l = float(U[0, 0])       # 左轮力矩
#     T_r = float(U[1, 0])       # 右轮力矩
#     T_pl = float(U[2, 0])      # 左腿摆力矩
#     T_pr = float(U[3, 0])      # 右腿摆力矩

#     # VMC 关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长 PD：保持已经验证的速度阻尼写法
#     dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl
#     dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr
#     F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
#     F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

#     T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
#     T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

#     T1_l = float(np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque))  # 左前
#     T1_r = float(np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque))  # 左后
#     T2_l = float(np.clip(T_JOINT_R[0, 0], -max_hip_torque, max_hip_torque))  # 右前
#     T2_r = float(np.clip(T_JOINT_R[1, 0], -max_hip_torque, max_hip_torque))  # 右后

#     # 最终赋值：当前模型轮力矩与 LQR 输出同号
#     motor_1.setTorque(T1_l)
#     motor_3.setTorque(T1_r)
#     motor_2.setTorque(T2_l)
#     motor_4.setTorque(T2_r)
#     motor_5.setTorque(float(np.clip(T_l, -max_wheel_torque, max_wheel_torque)))
#     motor_6.setTorque(float(np.clip(T_r, -max_wheel_torque, max_wheel_torque)))

#     # 每 100 ms 打印一次
#     counter += 1
#     if counter >= print_interval:
#         counter = 0
#         print(f"t={current_time:.2f}s, target_L0={target_L0_l:.4f} m")
#         print(
#             f"目标：s={float(expect_state[0, 0]):+.3f} m, "
#             f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
#             f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
#         )
#         print(
#             f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, "
#             f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
#             f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
#         )
#         print(
#             f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
#         )
#         print(f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m")
#         print(
#             f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
#             f"髋命令：M1={T1_l:+.2f}, M3={T1_r:+.2f}, M2={T2_l:+.2f}, M4={T2_r:+.2f} N*m"
#         )
#         print("--------------------------------")





















# from IMU import IMU_yaw
# import math
# import lagrange
# import leg
# from controller import Motor, PositionSensor, Gyro, Accelerometer, Robot, InertialUnit
# import mymath
# import numpy as np

# robot = Robot()
# timestep = int(robot.getBasicTimeStep())

# gyro = Gyro("gyro")
# imu = InertialUnit("imu")
# accelerometer = Accelerometer("accelerometer")
# robot_yaw = IMU_yaw()

# imu.enable(timestep)
# gyro.enable(timestep)
# accelerometer.enable(timestep)


# # 髋电机
# motor_1 = Motor("Left_Front_Motor")  # 左腿第一个主动关节
# motor_3 = Motor("Left_Back_Motor")
# motor_2 = Motor("Right_Front_Motor")  # 右腿第一个主动关节
# motor_4 = Motor("Right_Back_Motor")

# # 足电机
# motor_5 = Motor("Left_Wheel")  # 安装反的，设置力矩要加负号
# motor_6 = Motor("Right_Wheel")  # 安装反的，设置力矩要加负号

# # 髋角度
# ps_1 = PositionSensor("Left_Front_Motor_Sensor")
# ps_3 = PositionSensor("Left_Back_Motor_Sensor")
# ps_2 = PositionSensor("Right_Front_Motor_Sensor")
# ps_4 = PositionSensor("Right_Back_Motor_Sensor")

# # 足角度
# ps_5 = PositionSensor("Left_Wheel_Sensor")  # 安装反的，读取数据要加负号
# ps_6 = PositionSensor("Right_Wheel_Sensor")  # 安装反的，读取数据要加负号


# # 初始化电机
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setPosition(0.0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, accelerometer, imu]:
#     sensor.enable(timestep)


# # 差分初始化
# d_t = timestep / 1000
# Theta_b = mymath.Discreteness(d_t)

# diff_yaw = mymath.Discreteness(d_t)
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# Roll = mymath.Discreteness(d_t)

# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# d_Ll = mymath.Discreteness(d_t)
# d_Lr = mymath.Discreteness(d_t)

# Ll = mymath.Discreteness(d_t)

# Ll.last_diff = 0.16000280

# theta_l1.last_diff = 3.03552063
# theta_r1.last_diff = 3.03552063
# theta_l4.last_diff = 0.10607202
# theta_r4.last_diff = 0.10607202

# target_L0_l = 0.38
# target_L0_r = 0.38

# counter = 0
# print_interval = max(1, int(100 / timestep))  # 每100ms打印一次


# # PID初始化
# F0_control_l = mymath.PID_control(400, 0, 4000, target_L0_l)
# F0_control_r = mymath.PID_control(400, 0, 4000, target_L0_r)

# # 轮子半径
# r = 0.06
# s = 0.0

# # 左腿两个电机采用位置控制
# motor_1.setVelocity(0.2)
# motor_3.setVelocity(0.2)
# motor_2.setVelocity(0.2)
# motor_4.setVelocity(0.2)
# # 绝对目标位置都是1 rad
# motor_1.setPosition(0.3)
# motor_3.setPosition(0.3)
# motor_2.setPosition(0.3)
# motor_4.setPosition(0.3)

# current_time = 0
# flag = 0
# expect_state = np.zeros((10, 1))

# while robot.step(timestep) != -1:
#     current_time = current_time + d_t

#     # 获取姿态角
#     roll, pitch, yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(yaw)

#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()
#     ax, ay, az = accelerometer.getValues()

#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()

#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = 1.570796 - phi0_l + theta_b
#     theta_lr = 1.570796 - phi0_r + theta_b

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()

#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     # 四个主动关节角速度，单位 rad/s
#     dot_phi_l1 = theta_l1.Diff(phi_l1)
#     dot_phi_l4 = theta_l4.Diff(phi_l4)
#     dot_phi_r1 = theta_r1.Diff(phi_r1)
#     dot_phi_r4 = theta_r4.Diff(phi_r4)

#     # leg.spd()第二个返回值是 -dot_phi0
#     # 也就是虚拟腿相对机身的摆动角速度
#     dot_L0_l, dot_Phi0_l_negative = leg.spd(
#         dot_phi_l1,
#         dot_phi_l4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_l1,
#         phi_l4,
#     )

#     dot_L0_r, dot_Phi0_r_negative = leg.spd(
#         dot_phi_r1,
#         dot_phi_r4,
#         0.21,
#         0.25,
#         0.25,
#         0.21,
#         0.0,
#         phi_r1,
#         phi_r4,
#     )

#     # 腿长伸缩加速度
#     ddot_L0_l = d_Ll.Diff(dot_L0_l)
#     ddot_L0_r = d_Lr.Diff(dot_L0_r)

#     dot_theta_ll = dot_Phi0_l_negative + dot_theta_b
#     dot_theta_lr = dot_Phi0_r_negative + dot_theta_b

#     # 位移与速度
#     # s = r * (theta_wl + theta_wr) / 2
#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_b = (
#         dot_s_wheel
#         + 0.5
#         * (
#             L0_l * dot_theta_ll * math.cos(theta_ll)
#             + L0_r * dot_theta_lr * math.cos(theta_lr)
#         )
#         + 0.5 * (dot_L0_l * math.sin(theta_ll) + dot_L0_r * math.sin(theta_lr))
#     )
#     s += dot_s_b * d_t

#     # 当前的状态
#     # current_state = np.matrix(
#     #     [
#     #         [s],
#     #         [dot_s_b],
#     #         [yaw],
#     #         [dot_yaw],
#     #         [theta_ll],
#     #         [dot_theta_ll],
#     #         [theta_lr],
#     #         [dot_theta_lr],
#     #         [theta_b],
#     #         [dot_theta_b],
#     #     ]
#     # )
#     current_state = np.matrix(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ]
#     )

#     # 计算虚拟腿力到两个髋电机力矩的转换矩阵
#     if 0 < current_time <= 3:
#         flag = 0
#     elif 3 < current_time < 1000:
#         expect_state = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
#         flag = 1
#     else:
#         break
#     K = lagrange.K(
#         target_L0_l, target_L0_r, 50, 1, 500, 1, 500, 1, 500, 1, 5000, 1, 1, 1, 1, 1
#     )

#     U = K * (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     T_wl = 0
#     T_wr = 0

#     # VMC关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长PID
#     pid_force_l = F0_control_l.position_pid(L0_l)
#     pid_force_r = F0_control_r.position_pid(L0_r)

#     F_l = -pid_force_l
#     F_r = -pid_force_r
#     # 第二项为虚拟腿摆动力矩，本次只测腿长，所以设为0
#     T_JOINT_L = JRM_L * np.matrix(
#         [
#             [F_l],
#             # [0],
#             # [T_ll],
#             [0],
#         ]
#     )

#     T_JOINT_R = JRM_R * np.matrix(
#         [
#             [F_r],
#             # [0],
#             # [T_lr],
#             [0],
#         ]
#     )

#     T1_l = float(T_JOINT_L[0, 0])
#     T1_r = float(T_JOINT_L[1, 0])
#     T2_l = float(T_JOINT_R[0, 0])
#     T2_r = float(T_JOINT_R[1, 0])

#     motor_1.setTorque(float(np.clip(T1_l, -25.0, 25.0)))
#     motor_3.setTorque(float(np.clip(T1_r, -25.0, 25.0)))
#     motor_2.setTorque(float(np.clip(T2_l, -25.0, 25.0)))
#     motor_4.setTorque(float(np.clip(T2_r, -25.0, 25.0)))
#     motor_5.setTorque(float(np.clip(-T_wl, -10.0, 10.0)))
#     motor_6.setTorque(float(np.clip(-T_wr, -10.0, 10.0)))

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         print(
#             f"theta_b={math.degrees(theta_b):+.2f}°, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f}°/s"
#         )

#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"theta_ll={math.degrees(theta_ll):+.2f}°, "
#             f"phi_l1={phi_l1:+.2f}rad, "
#             f"phi_l4={phi_l4:+.2f}rad, "
#             f"ps_1={ps_1.getValue():+.2f}rad, "
#             f"ps_4={ps_4.getValue():+.2f}rad, "
#             f"dot_theta_ll={math.degrees(dot_theta_ll):+.2f}°/s,"
#             f"T1_l={T1_l}N, "
#             f"T1_r={T1_r}N"
#         )

#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"theta_lr={math.degrees(theta_lr):+.2f}°, "
#             f"dot_theta_lr={math.degrees(dot_theta_lr):+.2f}°/s"
#             f"T2_l={T2_l}N, "
#             f"T2_r={T2_r}N"
#         )

#         print(
#             f"phi0_l={math.degrees(phi0_l):+.4f} °, "
#             f"phi0_r={math.degrees(phi0_r):+.4f} °, "
#             f"flag={flag} , "
#             f"pid_force_l={pid_force_l:+.2f}N, "
#             f"pid_force_r={pid_force_r:+.2f}N"
#         )

#         print("--------------------------------")


















# """Webots 五连杆腿摆 LQR 极性测试。

# 测试流程：
# 1. 0～3 秒：四个髋电机使用位置控制，让左右虚拟腿同时偏转。
# 2. 3 秒以后：髋电机只切换一次到力矩控制，由 LQR 使腿摆角回到 0。
# 3. 同时以小力矩缓慢启用两个轮子，用于验证轮子输出极性。

# 运行本测试前，请在 Webots 中把 Robot 根节点设置为 locked TRUE。
# """

# import math

# import numpy as np
# from controller import Robot

# from IMU import IMU_yaw
# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# PERTURB_TIME = 3.0                 # 前 3 秒制造腿摆偏角
# PERTURB_MOTOR_POSITION = 0.50      # rad，约 17.2 度
# POSITION_SPEED = 0.20              # rad/s

# # 极性测试必须从小力矩开始，确认方向正确后再逐步增大。
# HIP_TORQUE_LIMIT = 0.30            # N*m

# # 当前测试没有启用腿长 PID，腿长仍接近初始的 0.16 m。
# LQR_MODEL_LEG_LENGTH = 0.16000280  # m

# # 已确认髋关节腿摆极性后，启用轮子做第二阶段测试。
# ENABLE_WHEEL_TORQUE = True
# WHEEL_TORQUE_LIMIT = 0.10          # N*m
# WHEEL_RAMP_TIME = 1.0              # s，轮力矩在 1 秒内由 0 缓慢增加


# robot = Robot()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)

# robot_yaw = IMU_yaw()


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)

# theta_l1_diff.last_diff = 3.03552063
# theta_r1_diff.last_diff = 3.03552063
# theta_l4_diff.last_diff = 0.10607202
# theta_r4_diff.last_diff = 0.10607202


# # ----------------------- 0～3 秒位置控制 -----------------------
# # 这里只发送位置命令。此阶段不能再调用四个髋电机的 setTorque()。
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(POSITION_SPEED)

# # 左右腿同时向同一腿摆方向偏转。
# motor_1.setPosition(PERTURB_MOTOR_POSITION)
# motor_3.setPosition(PERTURB_MOTOR_POSITION)
# motor_2.setPosition(PERTURB_MOTOR_POSITION)
# motor_4.setPosition(PERTURB_MOTOR_POSITION)

# # 轮子一直使用力矩模式。
# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# # K 只计算一次，避免每个仿真周期重复求解 Riccati 方程。
# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print("控制器启动：前 3 秒位置控制，3 秒后切换到腿摆 LQR。", flush=True)


# while robot.step(timestep) != -1:
#     current_time = robot.getTime()

#     # IMU：模型 x 向前、z 向上时，pitch/绕 y 轴角速度对应机体俯仰。
#     roll, pitch, raw_yaw = imu.getRollPitchYaw()
#     theta_b = pitch
#     yaw = robot_yaw.round_yaw(raw_yaw)
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 主动关节角转换到五连杆运动学角度。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     # leg.spd() 的第二个输出是 -dot(phi0)。
#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 保留你当前模型的轮子坐标约定：控制坐标 = -PositionSensor 原始值。
#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 本测试假设其余 6 个状态为 0，只把两条腿的摆角和角速度送入 LQR。
#     current_state = np.array(
#         [
#             [0.0],
#             [0.0],
#             [0.0],
#             [0.0],
#             [theta_ll],
#             [dot_theta_ll],
#             [theta_lr],
#             [dot_theta_lr],
#             [0.0],
#             [0.0],
#         ],
#         dtype=float,
#     )

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 把虚拟腿摆力矩映射到四个髋关节电机。
#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     # Webots 原始轮电机命令。模型轮力矩与电机命令之间保留一个负号。
#     wheel_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if current_time < PERTURB_TIME:
#         phase = "POSITION"

#         # 关键：这里不能调用 motor_1～motor_4.setTorque()。
#         # 四个髋电机会继续完成循环外设置的 0.30 rad 位置目标。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     else:
#         phase = "LQR"

#         # 到达 3 秒时只执行一次位置模式 -> 力矩模式切换。
#         if not released:
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             released = True
#             print("3 秒到达：已释放位置控制，开始 LQR 回中。", flush=True)

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)

#         if ENABLE_WHEEL_TORQUE:
#             # 切换后缓慢投入轮力矩，防止瞬间冲击。
#             wheel_ramp = float(
#                 np.clip((current_time - PERTURB_TIME) / WHEEL_RAMP_TIME, 0.0, 1.0)
#             )
#             cmd_wl = float(
#                 np.clip(T_wl * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * wheel_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT) * 5
#             )
#             motor_5.setTorque(cmd_wl)
#             motor_6.setTorque(cmd_wr)
#         else:
#             motor_5.setTorque(0.0)
#             motor_6.setTorque(0.0)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "LQR":
#             left_trend = "回中" if theta_ll * dot_theta_ll < 0.0 else "离开/暂未运动"
#             right_trend = "回中" if theta_lr * dot_theta_lr < 0.0 else "离开/暂未运动"

#             if abs(T_wl) < 1e-4:
#                 left_wheel_polarity = "力矩接近0"
#             elif T_wl * dot_theta_wl > 0.0:
#                 left_wheel_polarity = "极性一致"
#             else:
#                 left_wheel_polarity = "反向/暂未转动"

#             if abs(T_wr) < 1e-4:
#                 right_wheel_polarity = "力矩接近0"
#             elif T_wr * dot_theta_wr > 0.0:
#                 right_wheel_polarity = "极性一致"
#             else:
#                 right_wheel_polarity = "反向/暂未转动"
#         else:
#             left_trend = "制造偏角"
#             right_trend = "制造偏角"
#             left_wheel_polarity = "尚未启用"
#             right_wheel_polarity = "尚未启用"

#         print(
#             f"t={current_time:5.2f}s  phase={phase}  "
#             f"theta_b={math.degrees(theta_b):+.2f} deg"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, "
#             f"Phi0={math.degrees(phi0_l):+.2f} deg, "
#             f"theta={theta_ll:+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s, "
#             f"phi_l1={phi_l1:+.2f} deg, "
#             f"phi_l4={phi_l4:+.2f} deg, "
#             f"判断={left_trend}"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f} m, "
#             f"Phi0={math.degrees(phi0_r):+.2f} deg, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s, "
#             f"phi_r1={math.degrees(phi_r1):+.2f} deg, "
#             f"phi_r4={math.degrees(phi_r4):+.2f} deg, "
#             f"判断={right_trend}"
#         )
#         print(
#             f"LQR：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"髋电机命令：M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"左轮：模型速度={dot_theta_wl:+.4f} rad/s, "
#             f"电机命令={cmd_wl:+.4f} N*m, 判断={left_wheel_polarity}"
#         )
#         print(
#             f"右轮：模型速度={dot_theta_wr:+.4f} rad/s, "
#             f"电机命令={cmd_wr:+.4f} N*m, 判断={right_wheel_polarity}, "
#             f"ramp={wheel_ramp:.2f}"
#         )
#         print("--------------------------------")










# """Webots 机体俯仰 LQR 极性测试。

# 状态向量中只有 theta_b、dot_theta_b 使用真实测量值，其余 8 个状态全部为 0。

# 通过 TEST_MODE 分别测试：
# - "LEG_ONLY"：只施加虚拟腿摆力矩，两个轮子的力矩严格为 0；
# - "WHEEL_ONLY"：只施加轮力矩，四个髋关节电机的力矩严格为 0；
# - "BOTH"：腿摆与轮子同时工作，仅用于前两项通过后的组合验证。

# 流程：
# 1. 0～3 秒：Supervisor 固定 Robot 根节点，并将机体设置为 5 度初始倾角；
#    四个髋关节保持零位，轮力矩为 0。
# 2. 3 秒以后：解锁 Robot，髋关节切换到力矩模式，由机体状态产生的
#    LQR 轮力矩和虚拟腿摆力矩开始工作。

# WBT 使用要求：
# - Robot 节点 supervisor TRUE
# - 轮子接触地面
# - Robot 的 locked 初始值 TRUE/FALSE 均可，代码会自动控制
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次运行选 "LEG_ONLY"，复位仿真后改成 "WHEEL_ONLY" 再运行一次。
# TEST_MODE = "BOTH"
# VALID_TEST_MODES = ("LEG_ONLY", "WHEEL_ONLY", "BOTH")

# if TEST_MODE not in VALID_TEST_MODES:
#     raise ValueError(f"TEST_MODE 必须是 {VALID_TEST_MODES} 之一")

# RELEASE_TIME = 3.0
# INITIAL_BODY_PITCH = math.radians(-5.0)

# LQR_MODEL_LEG_LENGTH = 0.16000280

# HIP_POSITION_SPEED = 0.20
# HIP_TORQUE_LIMIT = 0.30
# WHEEL_TORQUE_LIMIT = 0.10
# TORQUE_RAMP_TIME = 0.50

# # 倾角超过该值就关闭全部力矩并重新固定机体，避免极性错误时摔飞。
# SAFETY_PITCH = math.radians(25.0)


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请把 Robot 的 supervisor 设置为 TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")

# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先固定机体，再设置绕 y 轴的初始俯仰角。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)

# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # ----------------------- 0～3 秒保持腿零位 -----------------------
# for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#     hip_motor.setVelocity(HIP_POSITION_SPEED)
#     hip_motor.setPosition(0.0)

# for wheel_motor in [motor_5, motor_6]:
#     wheel_motor.setPosition(float("inf"))
#     wheel_motor.setVelocity(0.0)
#     wheel_motor.setTorque(0.0)


# # --------------------------- 轮速差分 ---------------------------
# # 保留你当前使用的轮子反馈坐标：theta_w = -PositionSensor。
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     LQR_MODEL_LEG_LENGTH,
#     LQR_MODEL_LEG_LENGTH,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )

# released = False
# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))

# print(
#     f"控制器启动：TEST_MODE={TEST_MODE}，机体固定在 5 度，"
#     "3 秒后释放并启动仅机体状态的 LQR。",
#     flush=True,
# )

# controller_start_time = robot.getTime()


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 运动学只用于把 LQR 的虚拟腿摆力矩映射到四个髋电机。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     theta_wl = -ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 10 维状态顺序：
#     # s, ds, yaw, dyaw, theta_ll, dtheta_ll,
#     # theta_lr, dtheta_lr, theta_b, dtheta_b
#     #
#     # 本测试只有最后两个机体状态不是 0。
#     current_state = np.zeros((10, 1))
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)

#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     # 只输入虚拟腿摆力矩，轴向力暂时为 0。
#     T_JOINT_L = JRM_L * np.matrix([[0.0], [T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[0.0], [T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     torque_ramp = 0.0
#     cmd_wl = 0.0
#     cmd_wr = 0.0

#     if elapsed_time < RELEASE_TIME:
#         phase = "HOLD"

#         # 髋电机继续执行零位位置控制；此处不能调用髋电机 setTorque()。
#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)

#     elif not safety_stopped:
#         phase = "BODY_LQR"

#         if not released:
#             # 髋关节切换到力矩模式。
#             for hip_motor in [motor_1, motor_2, motor_3, motor_4]:
#                 hip_motor.setPosition(float("inf"))
#                 hip_motor.setVelocity(0.0)

#             # 清除固定阶段的物理速度后释放机体。
#             robot_node.resetPhysics()
#             locked_field.setSFBool(False)
#             released = True

#             print("3 秒到达：机体已释放，启动机体俯仰 LQR。", flush=True)

#         torque_ramp = float(
#             np.clip((elapsed_time - RELEASE_TIME) / TORQUE_RAMP_TIME, 0.0, 1.0)
#         )

#         # 根据 TEST_MODE 只允许被测执行器产生实际力矩。
#         if TEST_MODE in ("LEG_ONLY", "BOTH"):
#             cmd_m1 *= torque_ramp
#             cmd_m3 *= torque_ramp
#             cmd_m2 *= torque_ramp
#             cmd_m4 *= torque_ramp
#         else:
#             # WHEEL_ONLY：髋关节已经进入力矩模式，但实际命令严格为 0。
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0

#         if TEST_MODE in ("WHEEL_ONLY", "BOTH"):
#             # 已按你的实测结果修正：LQR轮力矩和Webots电机命令同号。
#             cmd_wl = float(
#                 np.clip(T_wl * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#             cmd_wr = float(
#                 np.clip(T_wr * torque_ramp, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)
#             )
#         else:
#             # LEG_ONLY：轮子保持力矩模式，但实际命令严格为 0。
#             cmd_wl = 0.0
#             cmd_wr = 0.0

#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#         if abs(theta_b) > SAFETY_PITCH:
#             for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#                 motor.setTorque(0.0)

#             locked_field.setSFBool(True)
#             safety_stopped = True
#             phase = "SAFETY_STOP"
#             cmd_m1 = 0.0
#             cmd_m3 = 0.0
#             cmd_m2 = 0.0
#             cmd_m4 = 0.0
#             cmd_wl = 0.0
#             cmd_wr = 0.0
#             print("安全停止：机体倾角超过 25 度，请检查极性。", flush=True)

#     else:
#         phase = "SAFETY_STOP"

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "BODY_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 body_trend = "接近中心"
#             elif theta_b * dot_theta_b < 0.0:
#                 body_trend = "正在回正"
#             else:
#                 body_trend = "离开中心/暂未运动"
#         elif phase == "HOLD":
#             body_trend = "保持初始倾角"
#         else:
#             body_trend = "已停止"

#         print(
#             f"t={elapsed_time:5.2f}s, mode={TEST_MODE}, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"判断={body_trend}"
#         )
#         print(
#             "输入状态："
#             f"theta_b={current_state[8, 0]:+.5f} rad, "
#             f"dot_theta_b={current_state[9, 0]:+.5f} rad/s，"
#             "其余状态全部为0"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"实际命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f}, "
#             f"M1={cmd_m1:+.4f}, M3={cmd_m3:+.4f}, "
#             f"M2={cmd_m2:+.4f}, M4={cmd_m4:+.4f} N*m"
#         )
#         print(
#             f"轮速：左={dot_theta_wl:+.3f}, 右={dot_theta_wr:+.3f} rad/s, "
#             f"ramp={torque_ramp:.2f}"
#         )
#         print("--------------------------------")


# """Webots 轮腿机器人 6 状态初步平衡测试。

# LQR 中仅使用以下 6 个真实状态：
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# s、dot_s、yaw、dot_yaw 全部置 0。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# # TARGET_LEG_LENGTH_L = 0.16000280
# # TARGET_LEG_LENGTH_R = 0.16000280
# TARGET_LEG_LENGTH_L = 0.2000280
# TARGET_LEG_LENGTH_R = 0.2000280

# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# WHEEL_TORQUE_LIMIT = 10.10    # N*m，第一次平衡测试从小力矩开始

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# if locked_field is None or rotation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     50, 1,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 6 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # 10维向量中只有腿摆4状态和机体2状态是真实值。
#     # s、dot_s、yaw、dot_yaw 全部为0。
#     current_state = np.zeros((10, 1))
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "SIX_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [theta_b, dot_theta_b, theta_ll, theta_lr, L0_l, L0_r]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "SIX_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "6状态输入："
#             f"[{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PID 保持，不属于这 6 个 LQR 状态。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器和腿长 PID。随后立即解锁，腿长 PID 和 6 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # 第一次落地测试先保持模型的初始腿长，避免固定根节点时强行伸腿顶地。
# TARGET_LEG_LENGTH_L = 0.16000280
# TARGET_LEG_LENGTH_R = 0.16000280
# # TARGET_LEG_LENGTH_L = 0.20
# # TARGET_LEG_LENGTH_R = 0.20


# # 腿长 PID 极性保持不变。mymath.PID_control 的 D 项没有除以 dt，
# # 因此 4000 在当前步长下过于激进，第一次联调先降为 500。
# LEG_KP = 400.0
# LEG_KI = 0.0
# LEG_KD = 500.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 3.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.210
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- 腿长 PID ---------------------------
# leg_length_pid_l = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_L
# )
# leg_length_pid_r = mymath.PID_control(
#     LEG_KP, LEG_KI, LEG_KD, TARGET_LEG_LENGTH_R
# )


# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K = lagrange.K(
#     TARGET_LEG_LENGTH_L,
#     TARGET_LEG_LENGTH_R,
#     500, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     5000, 1,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 用真实初始误差同时初始化 err 和 last_err，消除腿长 PID 的首拍 D 冲击。
# initial_leg_error_l = TARGET_LEG_LENGTH_L - L0_l_init
# initial_leg_error_r = TARGET_LEG_LENGTH_R - L0_r_init
# leg_length_pid_l.err = initial_leg_error_l
# leg_length_pid_l.last_err = initial_leg_error_l
# leg_length_pid_r.err = initial_leg_error_r
# leg_length_pid_r.last_err = initial_leg_error_r

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PID 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长PID输出。沿用你已经验证过的虚拟轴向力符号。
#     pid_force_l = leg_length_pid_l.position_pid(L0_l)
#     pid_force_r = leg_length_pid_r.position_pid(L0_r)

#     F_l = float(
#         np.clip(-pid_force_l - GRAVITY_FORCE_L, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )
#     F_r = float(
#         np.clip(-pid_force_r - GRAVITY_FORCE_R, -LEG_FORCE_LIMIT, LEG_FORCE_LIMIT)
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{TARGET_LEG_LENGTH_L:.4f} m, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{TARGET_LEG_LENGTH_R:.4f} m, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m"
#         )
#         print("--------------------------------")


# """由已能站立的 6 状态控制器扩展得到的 8 状态平衡与变腿长测试。

# LQR 中使用以下 8 个真实状态：
#     s, dot_s,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# yaw、dot_yaw 仍然置 0。除增加 s、dot_s 外，原来已经能站立的
# 腿长 PID、LQR 参数、VMC 映射和电机输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 8 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到 0.20 m，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 8 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 6状态稳态实际约需 0.16 N*m；8状态首次极性测试不能放出 10.10 N*m。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.220
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、腿摆角、轮角以及世界位置全部作为里程零点。
# _, theta_b_init, _ = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与 8 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 机体俯仰状态。
#     roll, theta_b, yaw = imu.getRollPitchYaw()
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 10维向量中恢复 s、dot_s，共8个真实状态；yaw、dot_yaw 保持0。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "EIGHT_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "EIGHT_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "8状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")


# """完整 10 状态轮腿平衡与变腿长测试。

# LQR 中使用以下 10 个真实状态：
#     s, dot_s, yaw, dot_yaw,
#     theta_ll, dot_theta_ll,
#     theta_lr, dot_theta_lr,
#     theta_b,  dot_theta_b

# 启动时的实际朝向作为 yaw=0，原来已经验证的腿长 PD、VMC 映射
# 以及轮子和髋关节输出极性保持不变。

# 腿长由独立 PD 保持，不属于这 10 个 LQR 状态。启动后先保持初始
# 0.16 m，再平滑升高到设定长度，避免目标阶跃引起上下振荡。

# 启动时只在根节点锁定状态下采集 1 个仿真周期，用真实传感器值初始化
# 差分器。随后立即解锁，腿长 PD 和 10 状态 LQR 同时工作，
# 没有延时释放或力矩渐入。

# WBT 使用要求：
# - WorldInfo.gravity 为 9.81 m/s^2 的正常重力；
# - Robot.supervisor TRUE；
# - 两个轮子正确接触水平地面。

# 控制使用编码器与腿部运动学计算出的 s、dot_s；Supervisor 世界 x
# 只用于在 Webots 中核对极性，不进入 LQR。
# """

# import math

# import numpy as np
# from controller import Supervisor

# import lagrange
# import leg
# import mymath


# # --------------------------- 测试参数 ---------------------------
# # 第一次落地联调先从 0 度开始。确认能支撑后再改为 +/-0.5 度测试扰动。
# # 不再直接从 2 度开始，避免绕 Robot 原点旋转后轮子瞬间压入地面。
# INITIAL_BODY_PITCH = math.radians(0.0)

# # 腿长到达终点后再原地转向 +pi/2。yaw 目标也使用 smoothstep，避免
# # 直接给 90 度阶跃造成左右轮差动力矩瞬间饱和。
# FINAL_YAW = math.pi / 2.0
# YAW_RAMP_TIME = 4.0

# # 当前 Webots 模型的轮编码器实测结果：原始 PositionSensor 正方向
# # 与世界 +x 前进方向一致，因此这里不再添加负号。
# WHEEL_RADIUS = 0.05995
# LEFT_WHEEL_SENSOR_SIGN = 1.0
# RIGHT_WHEEL_SENSOR_SIGN = 1.0

# # LQR 从启动第一拍就工作；腿长目标先保持 0.16 m，再平滑升到 0.20 m。
# START_LEG_LENGTH = 0.16000280
# FINAL_LEG_LENGTH = 0.22000000
# LEG_LENGTH_HOLD_TIME = 2.0
# LEG_LENGTH_RAMP_TIME = 3.0
# YAW_HOLD_TIME = LEG_LENGTH_HOLD_TIME + LEG_LENGTH_RAMP_TIME

# # 直接使用测得的 dot_L0 做阻尼，单位为 N*s/m。
# LEG_KP = 400.0
# LEG_KD_SPEED = 40.0

# # 先用小前馈验证支撑力极性，再按 5 -> 8 -> 12 -> 16 N 逐级增加。
# # 24.888 N 在 L0 约 0.16 m 的折叠姿态下会通过 JRM 产生较大髋力矩。
# GRAVITY_FORCE_L = 22.0
# GRAVITY_FORCE_R = 22.0

# LEG_FORCE_LIMIT = 40.0       # N，虚拟腿轴向力限幅
# HIP_TORQUE_LIMIT = 10.0       # N*m，包含腿长支撑与腿摆力矩
# # 完整状态首次联调仍保留轮力矩限幅，避免偏航或姿态误差导致瞬时冲击。
# WHEEL_TORQUE_LIMIT = 0.50

# SAFETY_PITCH = math.radians(20.0)
# SAFETY_YAW = math.radians(120.0)
# SAFETY_LEG_SWING = math.radians(30.0)
# SAFETY_LEG_SPEED = math.radians(300.0)
# SAFETY_LEG_LENGTH_MIN = 0.120
# SAFETY_LEG_LENGTH_MAX = 0.240
# SAFETY_POSITION = 0.50


# robot = Supervisor()
# timestep = int(robot.getBasicTimeStep())
# dt = timestep / 1000.0


# # --------------------------- Supervisor ---------------------------
# robot_node = robot.getSelf()
# if robot_node is None:
#     raise RuntimeError("无法获得 Robot 自身节点，请设置 supervisor TRUE")

# locked_field = robot_node.getField("locked")
# rotation_field = robot_node.getField("rotation")
# translation_field = robot_node.getField("translation")
# if locked_field is None or rotation_field is None or translation_field is None:
#     raise RuntimeError("无法访问 Robot 的 locked/rotation/translation 字段")

# # 先保持根节点锁定。设备启用后采集一拍真实初值，再立即解锁。
# locked_field.setSFBool(True)
# rotation_field.setSFRotation([0.0, 1.0, 0.0, INITIAL_BODY_PITCH])
# robot_node.resetPhysics()


# # --------------------------- 获取设备 ---------------------------
# imu = robot.getDevice("imu")
# gyro = robot.getDevice("gyro")

# motor_1 = robot.getDevice("Left_Front_Motor")
# motor_3 = robot.getDevice("Left_Back_Motor")
# motor_2 = robot.getDevice("Right_Front_Motor")
# motor_4 = robot.getDevice("Right_Back_Motor")
# motor_5 = robot.getDevice("Left_Wheel")
# motor_6 = robot.getDevice("Right_Wheel")

# ps_1 = robot.getDevice("Left_Front_Motor_Sensor")
# ps_3 = robot.getDevice("Left_Back_Motor_Sensor")
# ps_2 = robot.getDevice("Right_Front_Motor_Sensor")
# ps_4 = robot.getDevice("Right_Back_Motor_Sensor")
# ps_5 = robot.getDevice("Left_Wheel_Sensor")
# ps_6 = robot.getDevice("Right_Wheel_Sensor")

# imu.enable(timestep)
# gyro.enable(timestep)
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6]:
#     sensor.enable(timestep)


# # 六个电机全部使用直接力矩模式。
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float("inf"))
#     motor.setVelocity(0.0)
#     motor.setTorque(0.0)


# # --------------------------- 差分器 ---------------------------
# theta_l1_diff = mymath.Discreteness(dt)
# theta_l4_diff = mymath.Discreteness(dt)
# theta_r1_diff = mymath.Discreteness(dt)
# theta_r4_diff = mymath.Discreteness(dt)
# theta_wl_diff = mymath.Discreteness(dt)
# theta_wr_diff = mymath.Discreteness(dt)
# world_x_diff = mymath.Discreteness(dt)

# # --------------------------- LQR ---------------------------
# expect_state = np.zeros((10, 1))

# K_START = lagrange.K(
#     START_LEG_LENGTH,
#     START_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     1000, 5,
#     10000, 10,
#     1, 1, 1, 1,
# )
# K_FINAL = lagrange.K(
#     FINAL_LEG_LENGTH,
#     FINAL_LEG_LENGTH,
#     1000, 5,
#     500, 1,
#     500, 1,
#     500, 1,
#     10000, 10,
#     1, 1, 1, 1,
# )


# # --------------------------- 初值同步 ---------------------------
# # 传感器 enable 后必须先 step 一次才能得到有效读数。这一拍根节点仍锁定，
# # 六个电机力矩均为 0；它只用于初始化，不是控制等待时间。
# if robot.step(timestep) == -1:
#     raise RuntimeError("初始化传感器失败：仿真在控制开始前已结束")

# phi_l1_init = 3.03552063 - ps_1.getValue()
# phi_l4_init = 0.10607202 - ps_3.getValue()
# phi_r1_init = 3.03552063 - ps_2.getValue()
# phi_r4_init = 0.10607202 - ps_4.getValue()

# _, _, L0_l_init, phi0_l_init = leg.getPhi(
#     phi_l1_init, phi_l4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )
# _, _, L0_r_init, phi0_r_init = leg.getPhi(
#     phi_r1_init, phi_r4_init, 0.21, 0.25, 0.25, 0.21, 0.0
# )

# # 初始机体角、偏航角、腿摆角、轮角以及世界位置全部作为零点。
# _, theta_b_init, yaw_init = imu.getRollPitchYaw()
# theta_ll_init = math.pi / 2.0 - phi0_l_init + theta_b_init
# theta_lr_init = math.pi / 2.0 - phi0_r_init + theta_b_init

# theta_wl_init = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
# theta_wr_init = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()

# s_leg_init = 0.5 * (
#     L0_l_init * math.sin(theta_ll_init)
#     + L0_r_init * math.sin(theta_lr_init)
# )
# world_x_init = translation_field.getSFVec3f()[0]

# # 用真实初始关节角初始化差分器，防止第一拍出现虚假的几百 deg/s。
# theta_l1_diff.last_diff = phi_l1_init
# theta_l4_diff.last_diff = phi_l4_init
# theta_r1_diff.last_diff = phi_r1_init
# theta_r4_diff.last_diff = phi_r4_init
# theta_wl_diff.last_diff = theta_wl_init
# theta_wr_diff.last_diff = theta_wr_init
# world_x_diff.last_diff = 0.0

# # 清除初始化采样期间产生的速度，然后解锁。下一次 step 就是第一拍闭环控制。
# robot_node.resetPhysics()
# locked_field.setSFBool(False)

# safety_stopped = False
# counter = 0
# print_interval = max(1, int(100 / timestep))
# controller_start_time = robot.getTime()

# print(
#     "初值同步完成，已解锁：腿长 PD 与完整 10 状态 LQR 立即开始工作。",
#     flush=True,
# )
# print(
#     f"初值：L0_l={L0_l_init:.6f} m, L0_r={L0_r_init:.6f} m, "
#     f"phi0_l={math.degrees(phi0_l_init):+.3f} deg, "
#     f"phi0_r={math.degrees(phi0_r_init):+.3f} deg",
#     flush=True,
# )


# while robot.step(timestep) != -1:
#     elapsed_time = robot.getTime() - controller_start_time

#     # smoothstep：保持段与结束段的目标速度均为 0，避免 4 cm 目标阶跃。
#     linear_ratio = float(np.clip(
#         (elapsed_time - LEG_LENGTH_HOLD_TIME) / LEG_LENGTH_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     leg_ramp_ratio = linear_ratio * linear_ratio * (3.0 - 2.0 * linear_ratio)
#     target_L0_l = START_LEG_LENGTH + leg_ramp_ratio * (
#         FINAL_LEG_LENGTH - START_LEG_LENGTH
#     )
#     target_L0_r = target_L0_l

#     # 随目标腿长在两组已求得的 LQR 增益间调度。
#     K = (1.0 - leg_ramp_ratio) * K_START + leg_ramp_ratio * K_FINAL
#     if elapsed_time < LEG_LENGTH_HOLD_TIME:
#         leg_length_phase = "HOLD"
#     elif linear_ratio < 1.0:
#         leg_length_phase = "RAMP"
#     else:
#         leg_length_phase = "FINAL"

#     # 5 秒前保持原朝向，5～9 秒平滑转到 +pi/2，之后保持 90 度。
#     yaw_linear_ratio = float(np.clip(
#         (elapsed_time - YAW_HOLD_TIME) / YAW_RAMP_TIME,
#         0.0,
#         1.0,
#     ))
#     yaw_ramp_ratio = (
#         yaw_linear_ratio * yaw_linear_ratio * (3.0 - 2.0 * yaw_linear_ratio)
#     )
#     target_yaw = FINAL_YAW * yaw_ramp_ratio

#     if elapsed_time < YAW_HOLD_TIME:
#         yaw_phase = "HOLD"
#     elif yaw_linear_ratio < 1.0:
#         yaw_phase = "TURN"
#     else:
#         yaw_phase = "FINAL"

#     expect_state[2, 0] = target_yaw

#     # 机体姿态。yaw 使用启动时朝向作为 0，并包角到 [-pi, pi]。
#     roll, theta_b, raw_yaw = imu.getRollPitchYaw()
#     yaw = math.atan2(
#         math.sin(raw_yaw - yaw_init),
#         math.cos(raw_yaw - yaw_init),
#     )
#     dot_roll, dot_theta_b, dot_yaw = gyro.getValues()

#     # 五连杆关节角。
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(
#         phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(
#         phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0.0
#     )

#     # 虚拟腿相对世界竖直方向的摆角。
#     theta_ll = math.pi / 2.0 - phi0_l + theta_b
#     theta_lr = math.pi / 2.0 - phi0_r + theta_b

#     dot_phi_l1 = theta_l1_diff.Diff(phi_l1)
#     dot_phi_l4 = theta_l4_diff.Diff(phi_l4)
#     dot_phi_r1 = theta_r1_diff.Diff(phi_r1)
#     dot_phi_r4 = theta_r4_diff.Diff(phi_r4)

#     dot_L0_l, negative_dot_phi0_l = leg.spd(
#         dot_phi_l1, dot_phi_l4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_l1, phi_l4,
#     )
#     dot_L0_r, negative_dot_phi0_r = leg.spd(
#         dot_phi_r1, dot_phi_r4,
#         0.21, 0.25, 0.25, 0.21, 0.0,
#         phi_r1, phi_r4,
#     )

#     dot_theta_ll = negative_dot_phi0_l + dot_theta_b
#     dot_theta_lr = negative_dot_phi0_r + dot_theta_b

#     # --------------------------- s 与 dot_s ---------------------------
#     theta_wl = LEFT_WHEEL_SENSOR_SIGN * ps_5.getValue()
#     theta_wr = RIGHT_WHEEL_SENSOR_SIGN * ps_6.getValue()
#     dot_theta_wl = theta_wl_diff.Diff(theta_wl)
#     dot_theta_wr = theta_wr_diff.Diff(theta_wr)

#     # 轮轴位移直接由“当前轮角 - 初始轮角”计算，不再积分 dot_s，
#     # 从而避免上一版出现 dot_s 为正但 s 继续变负的累计错误。
#     s_wheel = WHEEL_RADIUS * (
#         (theta_wl - theta_wl_init) + (theta_wr - theta_wr_init)
#     ) / 2.0

#     # 机体相对轮轴的水平位移，并减去启动时的初值作为零点。
#     s_leg = 0.5 * (
#         L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)
#     ) - s_leg_init

#     s = s_wheel + s_leg

#     dot_s_wheel = WHEEL_RADIUS * (dot_theta_wl + dot_theta_wr) / 2.0
#     dot_s_leg = 0.5 * (
#         L0_l * dot_theta_ll * math.cos(theta_ll)
#         + dot_L0_l * math.sin(theta_ll)
#         + L0_r * dot_theta_lr * math.cos(theta_lr)
#         + dot_L0_r * math.sin(theta_lr)
#     )
#     dot_s = dot_s_wheel + dot_s_leg

#     # 仅用于验证里程计极性，不参与反馈。
#     s_world = translation_field.getSFVec3f()[0] - world_x_init
#     dot_s_world = world_x_diff.Diff(s_world)

#     # 完整 10 状态，顺序必须与 lagrange.py 建模顺序完全一致。
#     current_state = np.zeros((10, 1))
#     current_state[0, 0] = s
#     current_state[1, 0] = dot_s
#     current_state[2, 0] = yaw
#     current_state[3, 0] = dot_yaw
#     current_state[4, 0] = theta_ll
#     current_state[5, 0] = dot_theta_ll
#     current_state[6, 0] = theta_lr
#     current_state[7, 0] = dot_theta_lr
#     current_state[8, 0] = theta_b
#     current_state[9, 0] = dot_theta_b

#     U = K @ (expect_state - current_state)
#     T_wl = float(U[0, 0])
#     T_wr = float(U[1, 0])
#     T_ll = float(U[2, 0])
#     T_lr = float(U[3, 0])

#     # 腿长 PD。腿伸长时 dot_L0>0，正速度反馈会抵消负向伸腿力。
#     length_error_l = target_L0_l - L0_l
#     length_error_r = target_L0_r - L0_r
#     F_l = float(
#         np.clip(
#             -LEG_KP * length_error_l
#             + LEG_KD_SPEED * dot_L0_l
#             - GRAVITY_FORCE_L,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )
#     F_r = float(
#         np.clip(
#             -LEG_KP * length_error_r
#             + LEG_KD_SPEED * dot_L0_r
#             - GRAVITY_FORCE_R,
#             -LEG_FORCE_LIMIT,
#             LEG_FORCE_LIMIT,
#         )
#     )

#     JRM_L = leg.Mat_JRM(
#         phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21
#     )
#     JRM_R = leg.Mat_JRM(
#         phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21
#     )

#     if not safety_stopped:
#         phase = "TEN_STATE_LQR"
#         applied_T_ll = T_ll
#         applied_T_lr = T_lr

#         # 轮子极性采用你已经通过追杆现象验证的同号映射。
#         cmd_wl = float(np.clip(T_wl, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#         cmd_wr = float(np.clip(T_wr, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT))
#     else:
#         phase = "SAFETY_STOP"
#         applied_T_ll = 0.0
#         applied_T_lr = 0.0
#         cmd_wl = 0.0
#         cmd_wr = 0.0

#     # 腿长PID支撑力与腿摆LQR力矩从第一个周期同时工作。
#     T_JOINT_L = JRM_L * np.matrix([[F_l], [applied_T_ll]])
#     T_JOINT_R = JRM_R * np.matrix([[F_r], [applied_T_lr]])

#     raw_m1 = float(T_JOINT_L[0, 0])
#     raw_m3 = float(T_JOINT_L[1, 0])
#     raw_m2 = float(T_JOINT_R[0, 0])
#     raw_m4 = float(T_JOINT_R[1, 0])

#     cmd_m1 = float(np.clip(raw_m1, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m3 = float(np.clip(raw_m3, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m2 = float(np.clip(raw_m2, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     cmd_m4 = float(np.clip(raw_m4, -HIP_TORQUE_LIMIT, HIP_TORQUE_LIMIT))
#     hip_saturated = any(
#         abs(raw_torque) >= HIP_TORQUE_LIMIT
#         for raw_torque in [raw_m1, raw_m3, raw_m2, raw_m4]
#     )

#     if safety_stopped:
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0

#     if not safety_stopped:
#         motor_1.setTorque(cmd_m1)
#         motor_3.setTorque(cmd_m3)
#         motor_2.setTorque(cmd_m2)
#         motor_4.setTorque(cmd_m4)
#         motor_5.setTorque(cmd_wl)
#         motor_6.setTorque(cmd_wr)

#     # 倾角、腿摆角/速度、腿长或数值异常时立即停止并固定机构。
#     state_is_finite = all(
#         math.isfinite(value)
#         for value in [
#             s, dot_s, s_world, dot_s_world, yaw, dot_yaw,
#             theta_b, dot_theta_b,
#             theta_ll, dot_theta_ll, theta_lr, dot_theta_lr,
#             L0_l, L0_r,
#         ]
#     )
#     leg_length_is_safe = (
#         SAFETY_LEG_LENGTH_MIN < L0_l < SAFETY_LEG_LENGTH_MAX
#         and SAFETY_LEG_LENGTH_MIN < L0_r < SAFETY_LEG_LENGTH_MAX
#     )
#     leg_swing_is_safe = (
#         abs(theta_ll) < SAFETY_LEG_SWING
#         and abs(theta_lr) < SAFETY_LEG_SWING
#         and abs(dot_theta_ll) < SAFETY_LEG_SPEED
#         and abs(dot_theta_lr) < SAFETY_LEG_SPEED
#     )

#     if (
#         not safety_stopped
#         and (
#             abs(theta_b) > SAFETY_PITCH
#             or abs(yaw) > SAFETY_YAW
#             or not state_is_finite
#             or not leg_length_is_safe
#             or not leg_swing_is_safe
#             or abs(s) > SAFETY_POSITION
#         )
#     ):
#         # 先固定根节点并清零物理速度，再把四个髋电机切到当前位置保持。
#         # 这样安全停止后闭链腿不会因四个髋力矩全为 0 而继续塌下。
#         locked_field.setSFBool(True)
#         robot_node.resetPhysics()

#         motor_5.setTorque(0.0)
#         motor_6.setTorque(0.0)
#         for hip_motor, hip_sensor in [
#             (motor_1, ps_1),
#             (motor_3, ps_3),
#             (motor_2, ps_2),
#             (motor_4, ps_4),
#         ]:
#             hip_motor.setVelocity(0.5)
#             hip_motor.setPosition(hip_sensor.getValue())

#         safety_stopped = True
#         phase = "SAFETY_STOP"
#         cmd_m1 = cmd_m2 = cmd_m3 = cmd_m4 = 0.0
#         cmd_wl = cmd_wr = 0.0
#         stop_reasons = []
#         if abs(theta_b) > SAFETY_PITCH:
#             stop_reasons.append("机体倾角")
#         if abs(yaw) > SAFETY_YAW:
#             stop_reasons.append("偏航角")
#         if not leg_length_is_safe:
#             stop_reasons.append("腿长")
#         if not leg_swing_is_safe:
#             stop_reasons.append("腿摆角/角速度")
#         if abs(s) > SAFETY_POSITION:
#             stop_reasons.append("里程")
#         if not state_is_finite:
#             stop_reasons.append("非有限数值")
#         print(f"安全停止：{'、'.join(stop_reasons)}超过范围。", flush=True)

#     counter += 1
#     if counter >= print_interval:
#         counter = 0

#         if phase == "TEN_STATE_LQR":
#             if abs(theta_b) < math.radians(0.5):
#                 balance_status = "机体接近直立"
#             elif theta_b * dot_theta_b < 0.0:
#                 balance_status = "机体正在回正"
#             else:
#                 balance_status = "机体离开中心/暂未运动"
#         else:
#             balance_status = "已安全停止"

#         print(
#             f"t={elapsed_time:5.2f}s, phase={phase}, "
#             f"theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s, "
#             f"状态={balance_status}"
#         )
#         print(
#             f"偏航计划：{yaw_phase}, ramp={yaw_ramp_ratio:.3f}, "
#             f"target={math.degrees(target_yaw):+.2f} deg"
#         )
#         print(
#             f"偏航反馈：yaw={math.degrees(yaw):+.2f} deg, "
#             f"error={math.degrees(target_yaw - yaw):+.2f} deg, "
#             f"dot_yaw={math.degrees(dot_yaw):+.2f} deg/s"
#         )
#         print(
#             f"腿长计划：{leg_length_phase}, ramp={leg_ramp_ratio:.3f}, "
#             f"target={target_L0_l:.4f} m"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f}/{target_L0_l:.4f} m, "
#             f"dot_L0={dot_L0_l:+.4f} m/s, "
#             f"theta={math.degrees(theta_ll):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_ll):+.2f} deg/s"
#         )
#         print(
#             f"右腿：L0={L0_r:.4f}/{target_L0_r:.4f} m, "
#             f"dot_L0={dot_L0_r:+.4f} m/s, "
#             f"theta={math.degrees(theta_lr):+.2f} deg, "
#             f"dtheta={math.degrees(dot_theta_lr):+.2f} deg/s"
#         )
#         print(
#             "10状态输入："
#             f"[{s:+.4f}, {dot_s:+.4f}, "
#             f"{yaw:+.4f}, {dot_yaw:+.4f}, "
#             f"{theta_ll:+.4f}, {dot_theta_ll:+.4f}, "
#             f"{theta_lr:+.4f}, {dot_theta_lr:+.4f}, "
#             f"{theta_b:+.4f}, {dot_theta_b:+.4f}]"
#         )
#         print(
#             f"里程：s={s:+.5f} m, dot_s={dot_s:+.5f} m/s；"
#             f"世界x={s_world:+.5f} m, dx={dot_s_world:+.5f} m/s"
#         )
#         print(
#             f"位移分量：wheel={s_wheel:+.5f} m, leg={s_leg:+.5f} m；"
#             f"速度分量：wheel={dot_s_wheel:+.5f}, leg={dot_s_leg:+.5f} m/s"
#         )

#         if abs(s) < 0.002 or abs(s_world) < 0.002:
#             position_polarity = "位移太小，暂不能判断"
#         elif s * s_world > 0.0:
#             position_polarity = "位移极性一致"
#         else:
#             position_polarity = "位移极性相反"

#         if abs(dot_s) < 0.002 or abs(dot_s_world) < 0.002:
#             velocity_polarity = "速度太小，暂不能判断"
#         elif dot_s * dot_s_world > 0.0:
#             velocity_polarity = "速度极性一致"
#         else:
#             velocity_polarity = "速度极性相反"

#         print(
#             f"编码器：WL={dot_theta_wl:+.4f}, WR={dot_theta_wr:+.4f} rad/s；"
#             f"判断：{position_polarity}，{velocity_polarity}"
#         )
#         print(
#             f"LQR输出：T_wl={T_wl:+.4f}, T_wr={T_wr:+.4f}, "
#             f"T_ll={T_ll:+.4f}, T_lr={T_lr:+.4f} N*m"
#         )
#         print(
#             f"轮力矩分量：平均={(T_wl + T_wr) / 2.0:+.4f}, "
#             f"差动={(T_wr - T_wl) / 2.0:+.4f} N*m"
#         )
#         print(
#             f"腿长力：F_l={F_l:+.2f}, F_r={F_r:+.2f} N；"
#             f"实际轮命令：WL={cmd_wl:+.4f}, WR={cmd_wr:+.4f} N*m"
#         )
#         print(
#             f"实际髋命令：M1={cmd_m1:+.3f}, M3={cmd_m3:+.3f}, "
#             f"M2={cmd_m2:+.3f}, M4={cmd_m4:+.3f} N*m, "
#             f"饱和={'是' if hip_saturated else '否'}"
#         )
#         print("--------------------------------")



























# # -*- coding: utf-8 -*-
# # my_controller.py：10 状态 LQR + 腿长 PD

# import math
# import numpy as np

# import lagrange
# import leg
# import mymath
# from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit

# robot = Supervisor()  # Webots 的 Robot 节点需设置 supervisor TRUE
# timestep = int(robot.getBasicTimeStep())
# d_t = timestep / 1000

# # 初始机体倾角
# initial_body_pitch = math.radians(0.0)
# robot_node = robot.getSelf()
# locked_field = robot_node.getField('locked')
# locked_field.setSFBool(True)
# robot_node.getField('rotation').setSFRotation([0, 1, 0, initial_body_pitch])
# robot_node.resetPhysics()

# # 足电机
# motor_5 = Motor('Left_Wheel')
# motor_6 = Motor('Right_Wheel')

# # 髋电机
# motor_3 = Motor('Left_Front_Motor')   # 左后
# motor_1 = Motor('Left_Back_Motor')  # 左前
# motor_4 = Motor('Right_Front_Motor')  # 右后
# motor_2 = Motor('Right_Back_Motor') # 右前


# # 足角度
# ps_5 = PositionSensor('Left_Wheel_Sensor')
# ps_6 = PositionSensor('Right_Wheel_Sensor')

# # 髋角度
# ps_1 = PositionSensor('Left_Back_Motor_Sensor')
# ps_3 = PositionSensor('Left_Front_Motor_Sensor')
# ps_2 = PositionSensor('Right_Back_Motor_Sensor')
# ps_4 = PositionSensor('Right_Front_Motor_Sensor')

# gyro = Gyro('gyro')
# imu = InertialUnit('imu')

# # 初始化电机：直接力矩控制
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float('inf'))
#     motor.setVelocity(0)
#     motor.setTorque(0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, imu]:
#     sensor.enable(timestep)

# # 差分初始化
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# # 腿长目标：先保持 2 秒，再用 0.5 秒从 0.16 m 平滑变到 0.22 m
# start_L0 = 0.16000280
# final_L0 = 0.22
# leg_hold_time = 2.0
# leg_change_time = 0.5

# # 腿长 PD 参数与支撑力前馈
# leg_kp = 400.0
# leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
# gravity_force_l = 22.0        # N
# gravity_force_r = 22.0        # N

# # 输出限幅
# max_leg_force = 40.0         # N
# max_hip_torque = 10.0        # N*m
# max_wheel_torque = 0.50      # N*m
# r = 0.05995                  # 轮半径，m

# # 两组 K 只在启动时计算，之后随腿长目标插值
# K_start = lagrange.K(start_L0, start_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)
# K_final = lagrange.K(final_L0, final_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)

# s_local = 0.0
# # 采集一拍真实初值，避免差分器第一拍产生虚假速度
# if robot.step(timestep) == -1:
#     raise RuntimeError('仿真在控制器初始化时结束')

# phi_l1 = 3.03552063 - ps_1.getValue()
# phi_l4 = 0.10607202 - ps_3.getValue()
# phi_r1 = 3.03552063 - ps_2.getValue()
# phi_r4 = 0.10607202 - ps_4.getValue()

# _, _, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
# _, _, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

# _, theta_b, fai_init = imu.getRollPitchYaw()
# theta_ll = math.pi / 2 - phi0_l + theta_b
# theta_lr = math.pi / 2 - phi0_r + theta_b

# theta_wl_init = ps_5.getValue()
# theta_wr_init = ps_6.getValue()
# s_leg_init = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr))

# theta_l1.last_diff = phi_l1
# theta_l4.last_diff = phi_l4
# theta_r1.last_diff = phi_r1
# theta_r4.last_diff = phi_r4
# Theta_wl.last_diff = theta_wl_init
# Theta_wr.last_diff = theta_wr_init

# # 解锁后立即进入闭环
# robot_node.resetPhysics()
# locked_field.setSFBool(False)
# start_time = robot.getTime()
# counter = 0
# print_interval = max(1, int(100 / timestep))
# print('初值同步完成，10 状态 LQR 开始工作。', flush=True)

# # Main loop:
# while robot.step(timestep) != -1:
#     current_time = robot.getTime() - start_time

#     # 机器人基本状态获取：当前模型用 pitch 和绕 y 轴角速度
#     roll, pitch, fai = imu.getRollPitchYaw()
#     theta_b = pitch
#     fai = math.atan2(math.sin(fai - fai_init), math.cos(fai - fai_init))
#     dot_roll, dot_theta_b, dot_fai = gyro.getValues()

#     # 五连杆关节角
#     phi_l1 = 3.03552063 - ps_1.getValue()
#     phi_l4 = 0.10607202 - ps_3.getValue()
#     phi_r1 = 3.03552063 - ps_2.getValue()
#     phi_r4 = 0.10607202 - ps_4.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = math.pi / 2 - phi0_l + theta_b
#     theta_lr = math.pi / 2 - phi0_r + theta_b

#     # 当前模型的轮编码器直接读取，不加负号
#     theta_wl = ps_5.getValue()
#     theta_wr = ps_6.getValue()
#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     omega1l = theta_l1.Diff(phi_l1)
#     omega4l = theta_l4.Diff(phi_l4)
#     omega1r = theta_r1.Diff(phi_r1)
#     omega4r = theta_r4.Diff(phi_r4)

#     # leg.spd 第二个返回值是 -dot(phi0)
#     L0_speedl, phi0_speedl = leg.spd(omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4)
#     L0_speedr, phi0_speedr = leg.spd(omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4)
#     dot_theta_ll = phi0_speedl + dot_theta_b
#     dot_theta_lr = phi0_speedr + dot_theta_b

#     # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
#     s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
#     s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
#     # s = s_wheel + s_leg

#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_leg = 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll)
#                       + L0_speedl * math.sin(theta_ll)
#                       + L0_r * dot_theta_lr * math.cos(theta_lr)
#                       + L0_speedr * math.sin(theta_lr))
#     dot_s_b = dot_s_wheel + dot_s_leg

#     # # 高速时退出位置反馈，低速时积分位移进行位置保持
#     if abs(dot_s_b) < 0.1:
#         s_local += dot_s_b * d_t
#     else:
#         s_local = 0.0

#     s = s_local

#     # 状态方程的十个状态量
#     real_state = np.matrix([[s],
#                             [dot_s_b],
#                             [fai],
#                             [dot_fai],
#                             [theta_ll],
#                             [dot_theta_ll],
#                             [theta_lr],
#                             [dot_theta_lr],
#                             [theta_b],
#                             [dot_theta_b]])

#     # 根据时间设置腿长目标，首尾速度为零
#     leg_ratio = float(np.clip((current_time - leg_hold_time) / leg_change_time, 0, 1))
#     leg_ratio = leg_ratio * leg_ratio * (3 - 2 * leg_ratio)
#     target_L0_l = start_L0 + leg_ratio * (final_L0 - start_L0)
#     target_L0_r = target_L0_l

#     # 根据时间规定目标状态，直接修改各段矩阵中的数值即可
#     # 顺序：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll,
#     #       theta_lr, dot_theta_lr, theta_b, dot_theta_b
#     # 腿长由上面的 target_L0_l / target_L0_r 单独控制
#     if 0 <= current_time <= 2:
#         # 0～2 秒：保持原位、原朝向，腿和机体保持直立
#         expect_state = np.matrix([[0],               # 位移，m
#                                   [0],               # 速度，m/s
#                                   [0],               # 偏航角，rad
#                                   [0],               # 偏航角速度，rad/s
#                                   [0],               # 左腿摆角，rad
#                                   [0],               # 左腿摆角速度，rad/s
#                                   [0],               # 右腿摆角，rad
#                                   [0],               # 右腿摆角速度，rad/s
#                                   [0],               # 机体俯仰角，rad
#                                   [0]])              # 机体俯仰角速度，rad/s

#     elif 2 < current_time <= 2.5:
#         # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
#         expect_state = np.matrix([[0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     else:
#         # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
#         expect_state = np.matrix([[0],
#                                   [0.5],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     K = (1 - leg_ratio) * K_start + leg_ratio * K_final

#     # 解算力矩
#     U = K * (expect_state - real_state)
#     T_l = float(U[0, 0])       # 左轮力矩
#     T_r = float(U[1, 0])       # 右轮力矩
#     T_pl = float(U[2, 0])      # 左腿摆力矩
#     T_pr = float(U[3, 0])      # 右腿摆力矩

#     # VMC 关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长 PD：保持已经验证的速度阻尼写法
#     dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl
#     dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr
#     F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
#     F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

#     T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
#     T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

#     T1_l = float(np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque))  # 左前
#     T1_r = float(np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque))  # 左后
#     T2_l = float(np.clip(T_JOINT_R[0, 0], -max_hip_torque, max_hip_torque))  # 右前
#     T2_r = float(np.clip(T_JOINT_R[1, 0], -max_hip_torque, max_hip_torque))  # 右后

#     # 最终赋值：当前模型轮力矩与 LQR 输出同号
#     motor_1.setTorque(T1_l)
#     motor_3.setTorque(T1_r)
#     motor_2.setTorque(T2_l)
#     motor_4.setTorque(T2_r)
#     motor_5.setTorque(float(np.clip(T_l, -max_wheel_torque, max_wheel_torque)))
#     motor_6.setTorque(float(np.clip(T_r, -max_wheel_torque, max_wheel_torque)))

#     # 每 100 ms 打印一次
#     counter += 1
#     if counter >= print_interval:
#         counter = 0
#         print(f"t={current_time:.2f}s, target_L0={target_L0_l:.4f} m")
#         print(
#             f"目标：s={float(expect_state[0, 0]):+.3f} m, "
#             f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
#             f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
#         )
#         print(
#             f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, "
#             f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
#             f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
#         )
#         print(
#             f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
#         )
#         print(f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m")
#         print(
#             f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
#             f"髋命令：M1={T1_l:+.2f}, M3={T1_r:+.2f}, M2={T2_l:+.2f}, M4={T2_r:+.2f} N*m"
#         )
#         print("--------------------------------")





















# # -*- coding: utf-8 -*-
# # my_controller.py：10 状态 LQR + 腿长 PD

# import math
# import numpy as np

# import lagrange
# import leg
# import mymath
# from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit

# robot = Supervisor()  # Webots 的 Robot 节点需设置 supervisor TRUE
# timestep = int(robot.getBasicTimeStep())
# d_t = timestep / 1000

# # 初始机体倾角
# initial_body_pitch = math.radians(0.0)
# robot_node = robot.getSelf()
# locked_field = robot_node.getField('locked')
# locked_field.setSFBool(True)
# robot_node.getField('rotation').setSFRotation([0, 1, 0, initial_body_pitch])
# robot_node.resetPhysics()

# # 足电机
# motor_5 = Motor('Left_Wheel')
# motor_6 = Motor('Right_Wheel')

# # 髋电机
# motor_1 = Motor('Left_Front_Motor')   # 左前
# motor_2 = Motor('Left_Back_Motor')    # 左后
# motor_3 = Motor('Right_Front_Motor')  # 右前
# motor_4 = Motor('Right_Back_Motor')   # 右后


# # 足角度
# ps_5 = PositionSensor('Left_Wheel_Sensor')
# ps_6 = PositionSensor('Right_Wheel_Sensor')

# # 髋角度
# ps_1 = PositionSensor('Left_Front_Motor_Sensor')
# ps_2 = PositionSensor('Left_Back_Motor_Sensor')
# ps_3 = PositionSensor('Right_Front_Motor_Sensor')
# ps_4 = PositionSensor('Right_Back_Motor_Sensor')

# gyro = Gyro('gyro')
# imu = InertialUnit('imu')

# # 初始化电机：直接力矩控制
# for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
#     motor.setPosition(float('inf'))
#     motor.setVelocity(0)
#     motor.setTorque(0)

# # 初始化传感器
# for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, imu]:
#     sensor.enable(timestep)

# # 差分初始化
# Theta_wl = mymath.Discreteness(d_t)
# Theta_wr = mymath.Discreteness(d_t)
# theta_l1 = mymath.Discreteness(d_t)
# theta_l4 = mymath.Discreteness(d_t)
# theta_r1 = mymath.Discreteness(d_t)
# theta_r4 = mymath.Discreteness(d_t)

# # 腿长目标：先保持 2 秒，再用 0.5 秒从 0.16 m 平滑变到 0.22 m
# start_L0 = 0.16000280
# final_L0 = 0.22
# leg_hold_time = 2.0
# leg_change_time = 0.5

# # 腿长 PD 参数与支撑力前馈
# leg_kp = 400.0
# leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
# gravity_force_l = 22.0        # N
# gravity_force_r = 22.0        # N

# # 输出限幅
# max_leg_force = 40.0         # N
# max_hip_torque = 10.0        # N*m
# max_wheel_torque = 0.50      # N*m
# r = 0.05995                  # 轮半径，m

# # 两组 K 只在启动时计算，之后随腿长目标插值
# K_start = lagrange.K(start_L0, start_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)
# K_final = lagrange.K(final_L0, final_L0,
#                     1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10, 1, 1, 1, 1)

# s_local = 0.0
# # 采集一拍真实初值，避免差分器第一拍产生虚假速度
# if robot.step(timestep) == -1:
#     raise RuntimeError('仿真在控制器初始化时结束')

# phi_l1 = 3.03552063 - ps_2.getValue()
# phi_l4 = 0.10607202 - ps_1.getValue()
# phi_r1 = 3.03552063 + ps_4.getValue()
# phi_r4 = 0.10607202 + ps_3.getValue()

# _, _, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
# _, _, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

# _, theta_b, fai_init = imu.getRollPitchYaw()
# theta_ll = math.pi / 2 - phi0_l + theta_b
# theta_lr = math.pi / 2 - phi0_r + theta_b

# theta_wl_init = ps_5.getValue()
# theta_wr_init = -ps_6.getValue()
# s_leg_init = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr))

# theta_l1.last_diff = phi_l1
# theta_l4.last_diff = phi_l4
# theta_r1.last_diff = phi_r1
# theta_r4.last_diff = phi_r4
# Theta_wl.last_diff = theta_wl_init
# Theta_wr.last_diff = theta_wr_init

# # 解锁后立即进入闭环
# robot_node.resetPhysics()
# locked_field.setSFBool(False)
# start_time = robot.getTime()
# counter = 0
# print_interval = max(1, int(100 / timestep))
# print('初值同步完成，10 状态 LQR 开始工作。', flush=True)

# # Main loop:
# while robot.step(timestep) != -1:
#     current_time = robot.getTime() - start_time

#     # 机器人基本状态获取：当前模型用 pitch 和绕 y 轴角速度
#     roll, pitch, fai = imu.getRollPitchYaw()
#     theta_b = pitch
#     fai = math.atan2(math.sin(fai - fai_init), math.cos(fai - fai_init))
#     dot_roll, dot_theta_b, dot_fai = gyro.getValues()

#     # 五连杆关节角
#     phi_l1 = 3.03552063 - ps_2.getValue()
#     phi_l4 = 0.10607202 - ps_1.getValue()
#     phi_r1 = 3.03552063 + ps_4.getValue()
#     phi_r4 = 0.10607202 + ps_3.getValue()

#     phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
#     phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

#     theta_ll = math.pi / 2 - phi0_l + theta_b
#     theta_lr = math.pi / 2 - phi0_r + theta_b

#     # 当前模型右轮编码器极性与左轮相反
#     theta_wl = ps_5.getValue()
#     theta_wr = -ps_6.getValue()
#     dot_theta_wl = Theta_wl.Diff(theta_wl)
#     dot_theta_wr = Theta_wr.Diff(theta_wr)

#     omega1l = theta_l1.Diff(phi_l1)
#     omega4l = theta_l4.Diff(phi_l4)
#     omega1r = theta_r1.Diff(phi_r1)
#     omega4r = theta_r4.Diff(phi_r4)

#     # leg.spd 第二个返回值是 -dot(phi0)
#     L0_speedl, phi0_speedl = leg.spd(omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4)
#     L0_speedr, phi0_speedr = leg.spd(omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4)
#     dot_theta_ll = phi0_speedl + dot_theta_b
#     dot_theta_lr = phi0_speedr + dot_theta_b

#     # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
#     s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
#     s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
#     # s = s_wheel + s_leg

#     dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
#     dot_s_leg = 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll)
#                       + L0_speedl * math.sin(theta_ll)
#                       + L0_r * dot_theta_lr * math.cos(theta_lr)
#                       + L0_speedr * math.sin(theta_lr))
#     dot_s_b = dot_s_wheel + dot_s_leg

#     # # 高速时退出位置反馈，低速时积分位移进行位置保持
#     if abs(dot_s_b) < 0.1:
#         s_local += dot_s_b * d_t
#     else:
#         s_local = 0.0

#     s = s_local

#     # 状态方程的十个状态量
#     real_state = np.matrix([[s],
#                             [dot_s_b],
#                             [fai],
#                             [dot_fai],
#                             [theta_ll],
#                             [dot_theta_ll],
#                             [theta_lr],
#                             [dot_theta_lr],
#                             [theta_b],
#                             [dot_theta_b]])

#     # 根据时间设置腿长目标，首尾速度为零
#     leg_ratio = float(np.clip((current_time - leg_hold_time) / leg_change_time, 0, 1))
#     leg_ratio = leg_ratio * leg_ratio * (3 - 2 * leg_ratio)
#     target_L0_l = start_L0 + leg_ratio * (final_L0 - start_L0)
#     target_L0_r = target_L0_l

#     # 根据时间规定目标状态，直接修改各段矩阵中的数值即可
#     # 顺序：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll,
#     #       theta_lr, dot_theta_lr, theta_b, dot_theta_b
#     # 腿长由上面的 target_L0_l / target_L0_r 单独控制
#     if 0 <= current_time <= 2:
#         # 0～2 秒：保持原位、原朝向，腿和机体保持直立
#         expect_state = np.matrix([[0],               # 位移，m
#                                   [0],               # 速度，m/s
#                                   [0],               # 偏航角，rad
#                                   [0],               # 偏航角速度，rad/s
#                                   [0],               # 左腿摆角，rad
#                                   [0],               # 左腿摆角速度，rad/s
#                                   [0],               # 右腿摆角，rad
#                                   [0],               # 右腿摆角速度，rad/s
#                                   [0],               # 机体俯仰角，rad
#                                   [0]])              # 机体俯仰角速度，rad/s

#     elif 2 < current_time <= 2.5:
#         # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
#         expect_state = np.matrix([[0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     else:
#         # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
#         expect_state = np.matrix([[0],
#                                   [0.5],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0],
#                                   [0]])

#     K = (1 - leg_ratio) * K_start + leg_ratio * K_final

#     # 解算力矩
#     U = K * (expect_state - real_state)
#     T_l = float(U[0, 0])       # 左轮力矩
#     T_r = float(U[1, 0])       # 右轮力矩
#     T_pl = float(U[2, 0])      # 左腿摆力矩
#     T_pr = float(U[3, 0])      # 右腿摆力矩

#     # VMC 关节电机映射
#     JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
#     JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

#     # 腿长 PD：保持已经验证的速度阻尼写法
#     dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl
#     dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr
#     F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
#     F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

#     #T_JOINT_l[0, 0]  # phi_l1 → ps_2 → 左后 motor_2
#     #T_JOINT_l[1, 0]  # phi_l4 → ps_1 → 左前 motor_1

#     T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
#     T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

#     Phi1_N_l = float(np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque))  # phi1 对应的关节力矩,也就是左后
#     Phi4_N_l = float(np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque))  # phi4 对应的关节力矩，左前
#     Phi1_N_r = float(np.clip(T_JOINT_R[0, 0], -max_hip_torque, max_hip_torque))  
#     Phi4_N_r = float(np.clip(T_JOINT_R[1, 0], -max_hip_torque, max_hip_torque))  

#     # 最终赋值
#     motor_1.setTorque(Phi4_N_l)
#     motor_2.setTorque(Phi1_N_l)
#     motor_3.setTorque(-Phi4_N_r)
#     motor_4.setTorque(-Phi1_N_r)
#     motor_5.setTorque(float(np.clip(T_l, -max_wheel_torque, max_wheel_torque)))
#     motor_6.setTorque(float(np.clip(-T_r, -max_wheel_torque, max_wheel_torque)))

#     # 每 100 ms 打印一次
#     counter += 1
#     if counter >= print_interval:
#         counter = 0
#         print(f"t={current_time:.2f}s, target_L0={target_L0_l:.4f} m")
#         print(
#             f"目标：s={float(expect_state[0, 0]):+.3f} m, "
#             f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
#             f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
#         )
#         print(
#             f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, "
#             f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
#         )
#         print(
#             f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
#             f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
#         )
#         print(
#             f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
#             f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
#         )
#         print(f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m")
#         print(
#             f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
#             f"髋命令：M1={Phi4_N_l:+.2f}, M2={Phi1_N_l:+.2f}, M3={-Phi4_N_r:+.2f}, M4={-Phi1_N_r:+.2f} N*m"
#         )
#         print("--------------------------------")




























# -*- coding: utf-8 -*-
# my_controller.py：10 状态 LQR + 腿长 PD

import math
import numpy as np

import leg
import mymath
from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit


# 离线拟合的 K 系数，顺序来自嵌入式 K_out[40][6]。
# 每行六项依次对应：1、LL、LR、LL^2、LL*LR、LR^2。
K_out = np.array([
    [-22.06707888, -67.42656221, 66.26791534, 85.46883825, -51.07522658, -33.73026526],
    [-22.06707888, 66.26791534, -67.42656221, -33.73026526, -51.07522658, 85.46883825],
    [1.81128956, 1.918430097, -6.903779121, -15.97948866, 5.877553862, 14.63516654],
    [1.81128956, -6.903779121, 1.918430097, 14.63516654, 5.877553862, -15.97948866],
    [-15.14824449, -36.01213967, 43.3816016, 50.72573489, -50.1747366, -17.39032908],
    [-15.14824449, 43.3816016, -36.01213967, -17.39032908, -50.1747366, 50.72573489],
    [1.127532909, 1.802381363, -5.049836224, -10.17270597, 4.159121855, 9.210517058],
    [1.127532909, -5.049836224, 1.802381363, 9.210517058, 4.159121855, -10.17270597],
    [-14.2129079, 44.78097974, -10.3699394, -45.60483449, 18.62040156, 7.031888916],
    [14.2129079, 10.3699394, -44.78097974, -7.031888916, -18.62040156, 45.60483449],
    [-9.124394637, -15.17038212, -13.36305601, 22.24292986, -9.218010917, 19.34417343],
    [9.124394637, 13.36305601, 15.17038212, -19.34417343, 9.218010917, -22.24292986],
    [-0.8583352685, 3.714950088, -1.860249311, -2.421034471, 0.07873071115, 2.149693601],
    [0.8583352685, 1.860249311, -3.714950088, -2.149693601, -0.07873071115, 2.421034471],
    [-0.76649499, -1.490757399, -1.288643591, 2.1160362, -1.934414922, 1.83371078],
    [0.76649499, 1.288643591, 1.490757399, -1.83371078, 1.934414922, -2.1160362],
    [-21.10818894, -56.77637423, 24.32966624, 29.52229624, 0.6434581288, -21.126693],
    [-7.943967638, -7.876531888, 14.57864888, 22.43559203, -51.90850677, 5.125060155],
    [11.26391159, -23.94594434, 9.664567312, 17.697819, 7.064781723, -9.850072917],
    [-9.033956107, 20.99292908, -11.63959623, -16.60425863, -5.26749962, 10.86838742],
    [-1.650701785, -9.532109784, 4.69679896, -2.673738393, 0.4258297222, -2.973557576],
    [-1.058751116, -1.30675036, -0.9983133093, 5.419001644, -19.77515195, 6.687335891],
    [0.8259123223, -1.438973907, -0.04263930171, -0.2445994134, 0.5939012772, 0.6729832946],
    [-0.6202118184, 1.696819884, -0.2981017385, -0.3323619642, 0.02426771144, -0.7729406213],
    [-7.943967638, 14.57864888, -7.876531888, 5.125060155, -51.90850677, 22.43559203],
    [-21.10818894, 24.32966624, -56.77637423, -21.126693, 0.6434581288, 29.52229624],
    [-9.033956107, -11.63959623, 20.99292908, 10.86838742, -5.26749962, -16.60425863],
    [11.26391159, 9.664567312, -23.94594434, -9.850072917, 7.064781723, 17.697819],
    [-1.058751116, -0.9983133093, -1.30675036, 6.687335891, -19.77515195, 5.419001644],
    [-1.650701785, 4.69679896, -9.532109784, -2.973557576, 0.4258297222, -2.673738393],
    [-0.6202118184, -0.2981017385, 1.696819884, -0.7729406213, 0.02426771144, -0.3323619642],
    [0.8259123223, -0.04263930171, -1.438973907, 0.6729832947, 0.5939012772, -0.2445994134],
    [-5.815221667, 13.45456478, 2.688118766, -9.05395661, -4.66044793, -0.8714407737],
    [-5.815221667, 2.688118766, 13.45456478, -0.8714407737, -4.66044793, -9.05395661],
    [-70.50930504, -7.613664102, 6.764223053, 7.502016187, 0.1143313835, -6.615552449],
    [-70.50930504, 6.764223053, -7.613664102, -6.615552449, 0.1143313835, 7.502016187],
    [-0.4099128423, 0.5281040788, 0.6833233638, -0.07700483127, -0.7495149851, -0.4374959054],
    [-0.4099128423, 0.6833233638, 0.5281040788, -0.4374959054, -0.7495149851, -0.07700483127],
    [-2.348315438, -0.4629323578, 0.3901121781, 0.389834588, 0.04328625931, -0.3442750316],
    [-2.348315438, 0.3901121781, -0.4629323578, -0.3442750316, 0.04328625931, 0.389834588],
], dtype=np.float32)


def get_offline_K(LL, LR):
    LL = np.float32(LL)
    LR = np.float32(LR)
    polynomial = np.array(
        [1.0, LL, LR, LL * LL, LL * LR, LR * LR],
        dtype=np.float32,
    )

    K = np.zeros((4, 10), dtype=np.float32)
    for col in range(10):
        for row in range(4):
            value = np.dot(K_out[col * 4 + row], polynomial)
            K[row, col] = 0.0 if abs(value) < 1e-3 else value

    return np.matrix(K)

robot = Supervisor()  # Webots 的 Robot 节点需设置 supervisor TRUE
timestep = int(robot.getBasicTimeStep())
d_t = timestep / 1000

# 初始机体倾角
initial_body_pitch = math.radians(0.0)
robot_node = robot.getSelf()
locked_field = robot_node.getField('locked')
locked_field.setSFBool(True)
robot_node.getField('rotation').setSFRotation([0, 1, 0, initial_body_pitch])
robot_node.resetPhysics()

# 足电机
motor_5 = Motor('Left_Wheel')
motor_6 = Motor('Right_Wheel')

# 髋电机
motor_1 = Motor('Left_Front_Motor')   # 左前
motor_2 = Motor('Left_Back_Motor')    # 左后
motor_3 = Motor('Right_Front_Motor')  # 右前
motor_4 = Motor('Right_Back_Motor')   # 右后


# 足角度
ps_5 = PositionSensor('Left_Wheel_Sensor')
ps_6 = PositionSensor('Right_Wheel_Sensor')

# 髋角度
ps_1 = PositionSensor('Left_Front_Motor_Sensor')
ps_2 = PositionSensor('Left_Back_Motor_Sensor')
ps_3 = PositionSensor('Right_Front_Motor_Sensor')
ps_4 = PositionSensor('Right_Back_Motor_Sensor')

gyro = Gyro('gyro')
imu = InertialUnit('imu')

# 初始化电机：直接力矩控制
for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
    motor.setTorque(0)

# 初始化传感器
for sensor in [ps_1, ps_2, ps_3, ps_4, ps_5, ps_6, gyro, imu]:
    sensor.enable(timestep)

# 差分初始化
Theta_wl = mymath.Discreteness(d_t)
Theta_wr = mymath.Discreteness(d_t)
theta_l1 = mymath.Discreteness(d_t)
theta_l4 = mymath.Discreteness(d_t)
theta_r1 = mymath.Discreteness(d_t)
theta_r4 = mymath.Discreteness(d_t)

# 腿长目标：先保持 2 秒，再用 0.5 秒从 0.16 m 平滑变到 0.22 m
start_L0 = 0.16000280
final_L0 = 0.22
leg_hold_time = 2.0
leg_change_time = 0.5

# 腿长 PD 参数与支撑力前馈
leg_kp = 400.0
leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
gravity_force_l = 22.0        # N
gravity_force_r = 22.0        # N

# 输出限幅
max_leg_force = 40.0         # N
max_hip_torque = 10.0        # N*m
max_wheel_torque = 0.50      # N*m
r = 0.05995                  # 轮半径，m

s_local = 0.0
# 采集一拍真实初值，避免差分器第一拍产生虚假速度
if robot.step(timestep) == -1:
    raise RuntimeError('仿真在控制器初始化时结束')

phi_l1 = 3.03552063 - ps_2.getValue()
phi_l4 = 0.10607202 - ps_1.getValue()
phi_r1 = 3.03552063 + ps_4.getValue()
phi_r4 = 0.10607202 + ps_3.getValue()

_, _, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
_, _, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

_, theta_b, fai_init = imu.getRollPitchYaw()
theta_ll = math.pi / 2 - phi0_l + theta_b
theta_lr = math.pi / 2 - phi0_r + theta_b

theta_wl_init = ps_5.getValue()
theta_wr_init = -ps_6.getValue()
s_leg_init = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr))

theta_l1.last_diff = phi_l1
theta_l4.last_diff = phi_l4
theta_r1.last_diff = phi_r1
theta_r4.last_diff = phi_r4
Theta_wl.last_diff = theta_wl_init
Theta_wr.last_diff = theta_wr_init

# 解锁后立即进入闭环
robot_node.resetPhysics()
locked_field.setSFBool(False)
start_time = robot.getTime()
counter = 0
print_interval = max(1, int(100 / timestep))
print('初值同步完成，离线拟合 K 的 10 状态 LQR 开始工作。', flush=True)

# Main loop:
while robot.step(timestep) != -1:
    current_time = robot.getTime() - start_time

    # 机器人基本状态获取：当前模型用 pitch 和绕 y 轴角速度
    roll, pitch, fai = imu.getRollPitchYaw()
    theta_b = pitch
    fai = math.atan2(math.sin(fai - fai_init), math.cos(fai - fai_init))
    dot_roll, dot_theta_b, dot_fai = gyro.getValues()

    # 五连杆关节角
    phi_l1 = 3.03552063 - ps_2.getValue()
    phi_l4 = 0.10607202 - ps_1.getValue()
    phi_r1 = 3.03552063 + ps_4.getValue()
    phi_r4 = 0.10607202 + ps_3.getValue()

    phi_l2, phi_l3, L0_l, phi0_l = leg.getPhi(phi_l1, phi_l4, 0.21, 0.25, 0.25, 0.21, 0)
    phi_r2, phi_r3, L0_r, phi0_r = leg.getPhi(phi_r1, phi_r4, 0.21, 0.25, 0.25, 0.21, 0)

    theta_ll = math.pi / 2 - phi0_l + theta_b
    theta_lr = math.pi / 2 - phi0_r + theta_b

    # 当前模型右轮编码器极性与左轮相反
    theta_wl = ps_5.getValue()
    theta_wr = -ps_6.getValue()
    dot_theta_wl = Theta_wl.Diff(theta_wl)
    dot_theta_wr = Theta_wr.Diff(theta_wr)

    omega1l = theta_l1.Diff(phi_l1)
    omega4l = theta_l4.Diff(phi_l4)
    omega1r = theta_r1.Diff(phi_r1)
    omega4r = theta_r4.Diff(phi_r4)

    # leg.spd 第二个返回值是 -dot(phi0)
    L0_speedl, phi0_speedl = leg.spd(omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4)
    L0_speedr, phi0_speedr = leg.spd(omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4)
    dot_theta_ll = phi0_speedl + dot_theta_b
    dot_theta_lr = phi0_speedr + dot_theta_b

    # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
    s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
    s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
    s = s_wheel + s_leg

    dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
    dot_s_leg = 0.5 * (L0_l * dot_theta_ll * math.cos(theta_ll)
                      + L0_speedl * math.sin(theta_ll)
                      + L0_r * dot_theta_lr * math.cos(theta_lr)
                      + L0_speedr * math.sin(theta_lr))
    dot_s_b = dot_s_wheel + dot_s_leg

    # # 高速时退出位置反馈，低速时积分位移进行位置保持
    # if abs(dot_s_b) < 0.1:
    #     s_local += dot_s_b * d_t
    # else:
    #     s_local = 0.0

    # s = s_local

    # 状态方程的十个状态量
    real_state = np.matrix([[s],
                            [dot_s_b],
                            [fai],
                            [dot_fai],
                            [theta_ll],
                            [dot_theta_ll],
                            [theta_lr],
                            [dot_theta_lr],
                            [theta_b],
                            [dot_theta_b]])

    # 根据时间设置腿长目标，首尾速度为零
    leg_ratio = float(np.clip((current_time - leg_hold_time) / leg_change_time, 0, 1))
    leg_ratio = leg_ratio * leg_ratio * (3 - 2 * leg_ratio)
    target_L0_l = start_L0 + leg_ratio * (final_L0 - start_L0)
    target_L0_r = target_L0_l

    # 根据时间规定目标状态，直接修改各段矩阵中的数值即可
    # 顺序：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll,
    #       theta_lr, dot_theta_lr, theta_b, dot_theta_b
    # 腿长由上面的 target_L0_l / target_L0_r 单独控制
    if 0 <= current_time <= 2:
        # 0～2 秒：保持原位、原朝向，腿和机体保持直立
        expect_state = np.matrix([[0],               # 位移，m
                                  [0],               # 速度，m/s
                                  [0],               # 偏航角，rad
                                  [0],               # 偏航角速度，rad/s
                                  [0],               # 左腿摆角，rad
                                  [0],               # 左腿摆角速度，rad/s
                                  [0],               # 右腿摆角，rad
                                  [0],               # 右腿摆角速度，rad/s
                                  [0],               # 机体俯仰角，rad
                                  [0]])              # 机体俯仰角速度，rad/s

    elif 2 < current_time <= 2.5:
        # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
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
        # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
        expect_state = np.matrix([[0],
                                  [0.5],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0],
                                  [0]])

    # 使用与单片机相同的离线六系数多项式，根据目标腿长重建 4×10 K。
    K = get_offline_K(target_L0_l, target_L0_r)

    # 解算力矩
    U = K * (expect_state - real_state)
    T_l = float(U[0, 0])       # 左轮力矩
    T_r = float(U[1, 0])       # 右轮力矩
    T_pl = float(U[2, 0])      # 左腿摆力矩
    T_pr = float(U[3, 0])      # 右腿摆力矩

    # VMC 关节电机映射
    JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
    JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

    # 腿长 PD：保持已经验证的速度阻尼写法
    dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl
    dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr
    F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
    F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

    #T_JOINT_l[0, 0]  # phi_l1 → ps_2 → 左后 motor_2
    #T_JOINT_l[1, 0]  # phi_l4 → ps_1 → 左前 motor_1

    T_JOINT_l = JRM_L * np.matrix([[F_bl], [T_pl]])
    T_JOINT_R = JRM_R * np.matrix([[F_br], [T_pr]])

    Phi1_N_l = float(np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque))  # phi1 对应的关节力矩,也就是左后
    Phi4_N_l = float(np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque))  # phi4 对应的关节力矩，左前
    Phi1_N_r = float(np.clip(T_JOINT_R[0, 0], -max_hip_torque, max_hip_torque))  
    Phi4_N_r = float(np.clip(T_JOINT_R[1, 0], -max_hip_torque, max_hip_torque))  

    # 最终赋值
    motor_1.setTorque(Phi4_N_l)
    motor_2.setTorque(Phi1_N_l)
    motor_3.setTorque(-Phi4_N_r)
    motor_4.setTorque(-Phi1_N_r)
    motor_5.setTorque(float(np.clip(T_l, -max_wheel_torque, max_wheel_torque)))
    motor_6.setTorque(float(np.clip(-T_r, -max_wheel_torque, max_wheel_torque)))


    # 每 100 ms 打印一次
    counter += 1
    if counter >= print_interval:
        counter = 0
        print(f"t={current_time:.2f}s, target_L0={target_L0_l:.4f} m")
        print(
            f"目标：s={float(expect_state[0, 0]):+.3f} m, "
            f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
            f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
        )
        print(
            f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, s_wheel = {s_wheel:.4f} m  "
            f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
        )
        print(
            f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
            f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
        )
        print(
            f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
            f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
        )
        print(f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m, WL_P = {ps_5.getValue()}, WR_P = {ps_6.getValue()} , dot_theta_wl = {dot_theta_wl} , dot_theta_wr = {dot_theta_wl}")
        print(
            f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
            f"髋命令：M1={Phi4_N_l:+.2f}, M2={Phi1_N_l:+.2f}, M3={-Phi4_N_r:+.2f}, M4={-Phi1_N_r:+.2f} N*m"
        )
        print("--------------------------------")
