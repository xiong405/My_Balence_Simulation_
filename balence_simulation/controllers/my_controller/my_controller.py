# # 连续LQR控制
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
#                                   [0],
#                                   [0.5],
#                                   [0],
#                                   [0.1],
#                                   [0],
#                                   [0.1],
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




























# 离散LQR
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
final_L0 = 0.25
leg_hold_time = 2.0
leg_change_time = 0.5

# 腿长 PD 参数与支撑力前馈
leg_kp = 300.0
leg_kd = 40.0                 # N*s/m，直接对腿长速度加阻尼
gravity_force_l = 22.0        # N
gravity_force_r = 22.0        # N

# 输出限幅
max_leg_force = 40.0         # N
max_hip_torque = 10.0        # N*m
max_wheel_torque = 0.50      # N*m
r = 0.05995                  # 轮半径，m

s_local = 0.0
s_yaw = 0.0
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

    if abs(dot_fai) < 0.1:
        s_yaw += dot_fai * d_t
    else:
        s_yaw = 0.0

    fai = s_yaw

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
        expect_state = np.matrix([[0.1],               # 位移，m
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
                                  [0],
                                  [0],
                                  [5],
                                  [0.1],
                                  [0],
                                  [0.1],
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
