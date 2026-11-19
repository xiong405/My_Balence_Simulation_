# 离散LQR
# my_controller.py：10 状态 LQR + 腿长 PD

import math
import numpy as np

import leg
import mymath
from controller import Motor, PositionSensor, Gyro, Supervisor, InertialUnit
from MPC import Height_Roll_MPC
from Increase_MPC import Height_Roll_MPC_inc
# 离线拟合的 K 系数，顺序来自嵌入式 K_out[40][6]。
# 每行六项依次对应：1、LL、LR、LL^2、LL*LR、LR^2。
K_out = np.array(
    [
        [-0.6723788612, -13.03776997, 5.721045533, 11.88831912, -1.703333814, -3.014442775],
        [-0.6723788612, 5.721045533, -13.03776997, -3.014442775, -1.703333814, 11.88831912],
        [18.90559758, 26.90088136, -53.71497732, -84.56605502, 64.53016348, 29.13535485],
        [18.90559758, -53.71497732, 26.90088136, 29.13535485, 64.53016348, -84.56605502],
        [-1.189254641, -14.08141317, 7.332733174, 14.11553065, -5.104061875, -3.079391829],
        [-1.189254641, 7.332733174, -14.08141317, -3.079391829, -5.104061875, 14.11553065],
        [24.05802728, 26.41735225, -70.51907953, -101.2964349, 98.25487597, 32.96651791],
        [24.05802728, -70.51907953, 26.41735225, 32.96651791, 98.25487597, -101.2964349],
        [-3.030250997, 7.890409002, -4.601671197, -4.674439185, -0.2597629622, 4.895603184],
        [3.030250997, 4.601671197, -7.890409002, -4.895603184, 0.2597629622, 4.674439185],
        [-2.674843709, -38.03911187, -8.543443761, 51.43509431, -22.47492909, 12.72107166],
        [2.674843709, 8.543443761, 38.03911187, -12.72107166, 22.47492909, -51.43509431],
        [-1.073805353, 3.16096134, -2.146551306, -1.316853638, -1.089138006, 2.589210672],
        [1.073805353, 2.146551306, -3.16096134, -2.589210672, 1.089138006, 1.316853638],
        [-1.001810756, -16.40551521, -0.201610003, 20.26688936, -7.68566619, 0.557123547],
        [1.001810756, 0.201610003, 16.40551521, -0.557123547, 7.68566619, -20.26688936],
        [-3.699147997, -43.92231242, 8.589893224, 30.54110217, 7.187388621, -9.070152315],
        [-1.863412794, -4.903985214, -1.930033223, 8.327952502, -11.55154471, 9.514168405],
        [82.35939487, -29.91232285, -29.9380033, -64.10033505, 49.54695485, 19.92909068],
        [5.02299029, -4.032055023, -29.74253959, -6.829380901, 8.777717872, -4.785307652],
        [-0.2520215212, -4.126718951, 0.7279448822, -0.01210781592, 1.168323661, -0.5713221994],
        [-0.1233383509, -0.009759145887, -0.8816127933, 1.265531964, -4.975119925, 2.183320848],
        [6.88156606, 5.47470881, -6.094017019, -11.40369166, 2.840055041, 3.918990037],
        [0.3471201254, 4.467305118, 2.306068088, -7.883266752, 13.08466974, -13.24518206],
        [-1.863412794, -1.930033223, -4.903985214, 9.514168405, -11.55154471, 8.327952502],
        [-3.699147997, 8.589893224, -43.92231242, -9.070152315, 7.187388621, 30.54110217],
        [5.02299029, -29.74253959, -4.032055023, -4.785307652, 8.777717872, -6.829380901],
        [82.35939487, -29.9380033, -29.91232285, 19.92909068, 49.54695485, -64.10033505],
        [-0.1233383509, -0.8816127933, -0.009759145887, 2.183320848, -4.975119925, 1.265531964],
        [-0.2520215212, 0.7279448822, -4.126718951, -0.5713221994, 1.168323661, -0.01210781592],
        [0.3471201254, 2.306068088, 4.467305118, -13.24518206, 13.08466974, -7.883266752],
        [6.88156606, -6.094017019, 5.47470881, 3.918990037, 2.840055041, -11.40369166],
        [-6.775362343, -0.2362542996, 10.0998759, 11.12274048, -8.048809542, -6.534881635],
        [-6.775362343, 10.0998759, -0.2362542996, -6.534881635, -8.048809542, 11.12274048],
        [-9.748276081, -119.3115428, 16.80315758, 84.87302107, 33.65881746, -17.6415818],
        [-9.748276081, 16.80315758, -119.3115428, -17.6415818, 33.65881746, 84.87302107],
        [-1.082876002, -0.08419334757, 2.170580153, 1.644967137, -1.671609128, -1.504872084],
        [-1.082876002, 2.170580153, -0.08419334757, -1.504872084, -1.671609128, 1.644967137],
        [0.5297127386, -14.52129325, -2.988863482, 6.748342048, 10.90883308, 1.165861132],
        [0.5297127386, -2.988863482, -14.52129325, 1.165861132, 10.90883308, 6.748342048],
    ],
    dtype=np.float32,
)
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
locked_field = robot_node.getField("locked")
locked_field.setSFBool(True)
robot_node.getField("rotation").setSFRotation([0, 1, 0, initial_body_pitch])
robot_node.resetPhysics()

# 足电机
motor_5 = Motor("Left_Wheel")
motor_6 = Motor("Right_Wheel")

# 髋电机
motor_1 = Motor("Left_Front_Motor")  # 左前
motor_2 = Motor("Left_Back_Motor")  # 左后
motor_3 = Motor("Right_Front_Motor")  # 右前
motor_4 = Motor("Right_Back_Motor")  # 右后


# 足角度
ps_5 = PositionSensor("Left_Wheel_Sensor")
ps_6 = PositionSensor("Right_Wheel_Sensor")

# 髋角度
ps_1 = PositionSensor("Left_Front_Motor_Sensor")
ps_2 = PositionSensor("Left_Back_Motor_Sensor")
ps_3 = PositionSensor("Right_Front_Motor_Sensor")
ps_4 = PositionSensor("Right_Back_Motor_Sensor")

gyro = Gyro("gyro")
imu = InertialUnit("imu")

# 初始化电机：直接力矩控制
for motor in [motor_1, motor_2, motor_3, motor_4, motor_5, motor_6]:
    motor.setPosition(float("inf"))
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
leg_kd = 40.0  # N*s/m，直接对腿长速度加阻尼
gravity_force_l = 56.2  # N
gravity_force_r = 56.2  # N

# 输出限幅
max_leg_force = 40.0  # N
max_hip_torque = 20.0  # N*m
max_wheel_torque = 0.50  # N*m
r = 0.05995  # 轮半径，m

s_local = 0.0
s_yaw = 0.0
# 采集一拍真实初值，避免差分器第一拍产生虚假速度
if robot.step(timestep) == -1:
    raise RuntimeError("仿真在控制器初始化时结束")

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
print("初值同步完成，离线拟合 K 的 10 状态 LQR 开始工作。", flush=True)


# 创建MPC对象
mpc = Height_Roll_MPC_inc(
    mass=10.7,
    roll_inertia=0.118461022154,
    half_track=0.206217,
    damping=6.0,
    F_min=20.0,
    F_max=150.0,
    F_rate_max=30.0,
    equilibrium_force=56.2,
    Q=[5000.0, 50.0, 5000.0, 50.0],
    R=[0.1, 0.1],
    sample_time=0.01,
    Horizon=15,
)

time_step = int(robot.getBasicTimeStep())
d_t = time_step / 1000.0  # 0.002 s

mpc_Ts = 0.01  # 0.01 s
mpc_update_steps = max(
    1,
    int(round(mpc_Ts / d_t)),
)

mpc_counter = 0

# MPC尚未运行时使用平衡支撑力
F_LMPC = gravity_force_l
F_RMPC = gravity_force_r
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
    L0_speedl, phi0_speedl = leg.spd(
        omega1l, omega4l, 0.21, 0.25, 0.25, 0.21, 0, phi_l1, phi_l4
    )
    L0_speedr, phi0_speedr = leg.spd(
        omega1r, omega4r, 0.21, 0.25, 0.25, 0.21, 0, phi_r1, phi_r4
    )
    dot_theta_ll = phi0_speedl + dot_theta_b
    dot_theta_lr = phi0_speedr + dot_theta_b

    # 机体位移：轮轴位移 + 机体相对轮轴的水平位移
    s_wheel = r * (theta_wl - theta_wl_init + theta_wr - theta_wr_init) / 2
    s_leg = 0.5 * (L0_l * math.sin(theta_ll) + L0_r * math.sin(theta_lr)) - s_leg_init
    s = s_wheel + s_leg

    dot_s_wheel = r * (dot_theta_wl + dot_theta_wr) / 2
    dot_s_leg = 0.5 * (
        L0_l * dot_theta_ll * math.cos(theta_ll)
        + L0_speedl * math.sin(theta_ll)
        + L0_r * dot_theta_lr * math.cos(theta_lr)
        + L0_speedr * math.sin(theta_lr)
    )
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
    real_state = np.matrix(
        [
            [s],
            [dot_s_b],
            [fai],
            [dot_fai],
            [theta_ll],
            [dot_theta_ll],
            [theta_lr],
            [dot_theta_lr],
            [theta_b],
            [dot_theta_b],
        ]
    )

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
        expect_state = np.matrix(
            [
                [0],  # 位移，m
                [0],  # 速度，m/s
                [0],  # 偏航角，rad
                [0],  # 偏航角速度，rad/s
                [0],  # 左腿摆角，rad
                [0],  # 左腿摆角速度，rad/s
                [0],  # 右腿摆角，rad
                [0],  # 右腿摆角速度，rad/s
                [0],  # 机体俯仰角，rad
                [0],
            ]
        )  # 机体俯仰角速度，rad/s

    elif 2 < current_time <= 2.5:
        # 2～2.5 秒：腿长平滑升到 0.22 m，10 个目标状态仍为 0
        expect_state = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

    else:
        # 2.5 秒以后：直接给 +90 度偏航目标，到达后保持该朝向
        expect_state = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

    # 使用与单片机相同的离线六系数多项式，根据目标腿长重建 4×10 K。
    K = get_offline_K(target_L0_l, target_L0_r)

    # 解算力矩
    U = K * (expect_state - real_state)
    T_l = float(U[0, 0])  # 左轮力矩
    T_r = float(U[1, 0])  # 右轮力矩
    T_pl = float(U[2, 0])  # 左腿摆力矩
    T_pr = float(U[3, 0])  # 右腿摆力矩

    # VMC 关节电机映射
    JRM_L = leg.Mat_JRM(phi0_l, phi_l1, phi_l2, phi_l3, phi_l4, L0_l, 0.21, 0.21)
    JRM_R = leg.Mat_JRM(phi0_r, phi_r1, phi_r2, phi_r3, phi_r4, L0_r, 0.21, 0.21)

    H = 0.5 * (L0_l + L0_r) * np.cos(roll)  # 当前机体高度
    H_dot = (0.5 * (L0_speedl + L0_speedr) * np.cos(roll)) - (
        0.5 * (L0_l + L0_r) * np.sin(roll) * dot_roll
    )
    x = np.array([roll, dot_roll, H, H_dot])  # 当前状态
    

    if 3.0 <= current_time < 6:
        disturbance_L = 0.0   # N干扰
        disturbance_R = 0.0    
        x_ref = np.array([0, 0, 0.3, 0])
    else:
        disturbance_L = 0.0
        disturbance_R = 0.0
        x_ref = np.array([0, 0, 0.35, 0])

    # F_LMPC, F_RMPC = mpc.solve_OSQP(x, x_ref)

    if mpc_counter % mpc_update_steps == 0:
        F_LMPC, F_RMPC = mpc.solve_OSQP(
            x,
            x_ref,
        )
    mpc_counter += 1


    F_bl = -F_LMPC + disturbance_L
    F_br = -F_RMPC + disturbance_R

    # 腿长 PD：保持已经验证的速度阻尼写法
    # dF_0_l = -leg_kp * (target_L0_l - L0_l) + leg_kd * L0_speedl  + disturbance_L
    # dF_0_r = -leg_kp * (target_L0_r - L0_r) + leg_kd * L0_speedr + disturbance_R
    # F_bl = float(np.clip(dF_0_l - gravity_force_l, -max_leg_force, max_leg_force))
    # F_br = float(np.clip(dF_0_r - gravity_force_r, -max_leg_force, max_leg_force))

    # T_JOINT_l[0, 0]  # phi_l1 → ps_2 → 左后 motor_2
    # T_JOINT_l[1, 0]  # phi_l4 → ps_1 → 左前 motor_1

    T_JOINT_l = JRM_L @ np.array([[F_bl], [T_pl]])
    T_JOINT_R = JRM_R @ np.array([[F_br], [T_pr]])

    Phi1_N_l = float(
        np.clip(T_JOINT_l[0, 0], -max_hip_torque, max_hip_torque)
    )  # phi1 对应的关节力矩,也就是左后

    Phi4_N_l = float(
        np.clip(T_JOINT_l[1, 0], -max_hip_torque, max_hip_torque)
    )  # phi4 对应的关节力矩，左前

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
        # print(
        #     f"目标：s={float(expect_state[0, 0]):+.3f} m, "
        #     f"dot_s={float(expect_state[1, 0]):+.3f} m/s, "
        #     f"yaw={math.degrees(float(expect_state[2, 0])):+.1f} deg"
        # )
        # print(
        #     f"s={s:+.4f} m, dot_s={dot_s_b:+.4f} m/s, s_wheel = {s_wheel:.4f} m  "
        #     f"yaw={math.degrees(fai):+.2f} deg, dot_yaw={math.degrees(dot_fai):+.2f} deg/s"
        # )
        # print(
        #     f"左腿：L0={L0_l:.4f} m, theta={math.degrees(theta_ll):+.2f} deg；"
        #     f"右腿：L0={L0_r:.4f} m, theta={math.degrees(theta_lr):+.2f} deg"
        # )
        # print(
        #     f"机体：theta_b={math.degrees(theta_b):+.2f} deg, "
        #     f"dot_theta_b={math.degrees(dot_theta_b):+.2f} deg/s"
        # )
        # print(
        #     f"LQR：WL={T_l:+.3f}, WR={T_r:+.3f}, TL={T_pl:+.3f}, TR={T_pr:+.3f} N*m, WL_P = {ps_5.getValue()}, WR_P = {ps_6.getValue()} , dot_theta_wl = {dot_theta_wl} , dot_theta_wr = {dot_theta_wl}"
        # )
        print(
            f"腿长力：FL={F_bl:+.2f}, FR={F_br:+.2f} N；"
            f"髋命令：M1={Phi4_N_l:+.2f}, M2={Phi1_N_l:+.2f}, M3={-Phi4_N_r:+.2f}, M4={-Phi1_N_r:+.2f} N*m"
        )
        print(
            f"H={H:.5f}, H_ref={x_ref[2]:.5f}, F_LMPC={F_LMPC:.5f}, F_RMPC={F_RMPC:.5f}, Roll = {roll:.5f}, Roll_dot = {dot_roll:.5f}"
        )
        print(
            f"H_error={x_ref[2] - H:+.5f}, "
            f"dFL={F_LMPC - mpc.F_eq:+.5f}, "
            f"dFR={F_RMPC - mpc.F_eq:+.5f}"
        )

        print("--------------------------------")
