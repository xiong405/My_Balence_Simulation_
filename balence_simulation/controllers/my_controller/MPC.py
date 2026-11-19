import numpy as np
import scipy.sparse as sparse
from scipy.signal import cont2discrete
import osqp
import matplotlib.pyplot as plt

class Height_Roll_MPC:
    def __init__(
        self,
        mass,  # 质量
        roll_inertia,  # roll转动惯量
        half_track,  # 机器人中心到左右轮子距离
        damping,  # 竖直等效阻尼
        F_min,  # 腿部力约束
        F_max,  # 腿部力约束
        F_rate_max,  # 力变化率约束
        equilibrium_force,  # 重力前馈力
        Q,  # 状态权重
        R,  # 左右腿增量支撑力权重
        sample_time=0.01,  # 仿真间隔时间
        Horizon=15,  # 预测步长
    ):

        # 1.系统维度
        self.nx = 4  # roll、roll_dot、H、H_dot
        self.nu = 2  # 左右腿支撑力
        self.N = int(Horizon)  # 预测未来步数
        self.Ts = float(sample_time)  # 仿真时间

        # 2.物理模型参数
        self.mass = float(mass)
        self.roll_inertia = float(roll_inertia)
        self.half_track = float(half_track)
        self.damping = float(damping)  # 竖直等效阻尼
        if equilibrium_force is None:  # 计算前馈力
            self.F_eq = self.mass * 9.81 / 2.0
        else:
            self.F_eq = float(equilibrium_force)

        # 3.力约束
        self.F_min = float(F_min)
        self.F_max = float(F_max)
        self.F_rate_max = float(F_rate_max)  # 力变化率约束


        # 4.Q,R权重
        Q = np.asarray(Q, dtype=float)
        R = np.asarray(R, dtype=float)
        # 如果传入的是对角线向量，自动转换成矩阵
        if Q.ndim == 1:  # 如果是一维度，则转化为对角矩阵
            Q = np.diag(Q)
        if R.ndim == 1:
            R = np.diag(R)
        self.Q = Q
        self.R = R

        # 5.建立连续状态空间模型
        self.Ac = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [
                    0.0,
                    0.0,
                    0.0,
                    -self.damping / self.mass,
                ],
            ]
        )
        self.Bc = np.array(
            [
                [0.0, 0.0],
                [
                    self.half_track / self.roll_inertia,
                    -self.half_track / self.roll_inertia,
                ],
                [0.0, 0.0],
                [
                    1.0 / self.mass,
                    1.0 / self.mass,
                ],
            ]
        )

        # 6.ZOH离散化
        self.Ad, self.Bd, _, _, _ = cont2discrete(
            (
                self.Ac,
                self.Bc,
                np.eye(self.nx),
                np.zeros((self.nx, self.nu)),
            ),
            self.Ts,
            method="zoh",
        )


        #获得M,C矩阵
        self.M,self.C = self.MPC_Prediction_M_C()
        #获取Q_ba,R_ba,H_qp
        self.Q_ba, self.R_ba, self.H_qp = (self.MPC_Prediction_Q_R_H_qp())



        self.u_previous = np.zeros(self.nu,dtype=float) # 初始化u_previous
        self.F_constraint = (self.MPC_Constraint_Matrix())# 完整约束矩阵 (60,30)
        (self.lower_constraint,self.upper_constraint,) = self.MPC_Force_Constraint()
        self.MPC_Init_OSQP()# 初始化OSQP


    #构造M和C矩阵
    def MPC_Prediction_M_C(self):
        # 构造MPC预测矩阵：X = M @ x(k) + C @ U
        M = np.zeros((self.N * self.nx, self.nx))  # (60, 4)
        C = np.zeros((self.N * self.nx,self.N * self.nu)) #*(60,30)
        for i in range(self.N):
            row_start = i * self.nx#构造行
            row_end = (i + 1) * self.nx
            M[row_start:row_end,:] = np.linalg.matrix_power(self.Ad, i+1)
            for j in range (i + 1):
                col_start = j * self.nu#构造列
                col_end = (j + 1) * self.nu
                C[row_start:row_end,col_start:col_end] = np.linalg.matrix_power(self.Ad, i - j) @ self.Bd
        return M ,C


    #构造Q,R,H,f矩阵
    def MPC_Prediction_Q_R_H_qp(self):
        Q_ba = np.kron(np.eye(self.N),self.Q)#(60,60)
        R_ba = np.kron(np.eye(self.N),self.R)#(30,30)
        H_qp = 2.0 * (self.C.T @ Q_ba @ self.C + R_ba)#(30,30)
        H_qp = 0.5 * (H_qp + H_qp.T)# 消除浮点计算造成的微小不对称
        return Q_ba, R_ba, H_qp


    #求解f_QP
    def MPC_Calculate_f_qp(self, x, x_ref):
        """f_QP = 2C_T @ Q @ (M @ x - Xref)"""
        x = np.asarray(x,dtype=float).reshape(self.nx)# 当前状态：(4,)通过reshape转化为1维度
        x_ref = np.asarray(x_ref,dtype=float).reshape(self.nx)# 单步参考状态：(4,)
        X_ref = np.tile(x_ref,self.N)# 未来N步参考状态：(60,)
        f_qp = 2.0 * (self.C.T @ self.Q_ba @ (self.M @ x - X_ref))#(30,)
        return np.asarray(f_qp,dtype=float,).reshape(-1)#最后输出一维度的


    #力矩约束函数,生成l,u(60,)用于约束的最大值和最小值
    def MPC_Force_Constraint(self): 
        nU = self.N * self.nu#(30)
        delta_F_min = self.F_min - self.F_eq
        delta_F_max = self.F_max - self.F_eq
        lower_force = np.full(nU,delta_F_min,dtype=float)#新建一个数组，全部元素都填同一个值delta_F_min
        upper_force = np.full(nU,delta_F_max,dtype=float)

        lower_rate = np.full(nU,-self.F_rate_max,dtype=float)#(30,)
        upper_rate = np.full(nU,self.F_rate_max,dtype=float)
        lower_rate[:2] = (self.u_previous - self.F_rate_max)#需要实实跟新(0:2)的(u_previous + min < u_current <  u_previous + max)
        upper_rate[:2] = (self.u_previous + self.F_rate_max)

        lower_constraint = np.concatenate((lower_force,lower_rate))#(60,)
        upper_constraint = np.concatenate((upper_force,upper_rate))
        return (lower_constraint,upper_constraint)


    #构造完整约束矩阵F_constraint，生成A矩阵
    def MPC_Constraint_Matrix(self):
        """        
        min_U  1/2 * U.T @ P @ U + q.T @ U

        s.t.   l <= A @ U <= u
        """

        nU = self.N * self.nu
        #最大力和最小力约束
        F_limit = np.eye(nU,dtype=float)#(30,30)
        F_rate = np.zeros((nU,nU))#(30,30)
        # 左右腿两个输入
        Iu = np.eye(self.nu,dtype=float) # (2,2)
        #构建F_rate矩阵
        """    
        [ 1,  0,  0,  0,  0,  0],#前两相的U矩阵是(u_previous + min < u_current <  u_previous + max)
        [ 0,  1,  0,  0,  0,  0],
        [-1,  0,  1,  0,  0,  0],
        [ 0, -1,  0,  1,  0,  0],
        [ 0,  0, -1,  0,  1,  0],
        [ 0,  0,  0, -1,  0,  1],
        """
        for i in range(self.N):
            current = slice(i * self.nu,(i + 1) * self.nu)
             # 当前控制量系数为+I     
            F_rate[current,current] = Iu#(0:2)是I矩阵
            if i > 0:
                previous = slice((i-1) * self.nu, i * self.nu)
                F_rate[current,previous] = -Iu
        F_constraint = np.vstack((F_limit,F_rate))
        return sparse.csc_matrix(F_constraint)#(60,30)




    #初始化QP求解器
    def MPC_Init_OSQP(self):
        nU = self.N * self.nu
        H_qp = sparse.csc_matrix(np.triu(self.H_qp))# (30,30)
        f_init = np.zeros(nU,dtype=float,)# (30,)    
        self.solver = osqp.OSQP()# 创建求解器对象
        self.solver.setup(
                P=H_qp,
                q=f_init,
                A=self.F_constraint,
                l=self.lower_constraint,
                u=self.upper_constraint,
                verbose=False,
            )

    
    #构造QP求解器

    def solve_OSQP(self, x, x_ref):
        f_qp = self.MPC_Calculate_f_qp(x,x_ref)#(30,),根据当前状态和目标修改f_qp
        (self.lower_constraint,self.upper_constraint,) = self.MPC_Force_Constraint()#获取限制力矩
        self.solver.update(q=f_qp, l=self.lower_constraint,u=self.upper_constraint)#更新OSQP的线性项q,l,u
        result = self.solver.solve()#求解QP

        U_out = np.asarray(result.x,dtype=float,).reshape(-1)  # (30,)
        u_now = U_out[:2]#取前两行(2,0)
        self.u_previous = u_now.copy()#更新上一次u_previous
        delta_F_left = U_out[0]
        delta_F_right = U_out[1]
        F_left = (self.F_eq + delta_F_left)
        F_right = (self.F_eq + delta_F_right)
        return (float(F_left),float(F_right),)



    def test_osqp_end_to_end(mpc):
        mpc.u_previous = np.zeros(
            mpc.nu,
            dtype=float,
        )

        x_ref = np.array([
            0.0,   # 目标roll
            0.0,   # 目标roll速度
            0.25,  # 目标高度
            0.0,   # 目标高度速度
        ])   

        x = np.array([
            0.0,   # 目标roll
            0.0,   # 目标roll速度
            0.20,  # 目标高度
            0.0,   # 目标高度速度
        ])  
        F_left,F_right = mpc.solve_OSQP(x,x_ref)
        return (float(F_left),float(F_right),)

def test_closed_loop(mpc):

    # 初始状态：存在roll和高度偏差
    x = np.array([
        0.1,   # roll
        0.0,   # roll_dot
        0.22,  # H
        0.0,   # H_dot
    ])

    x_ref = np.array([
        0.0,
        0.0,
        0.25,
        0.0,
    ])

    for k in range(300):

        # MPC返回实际支撑力
        F_left, F_right = mpc.solve_OSQP(
            x,
            x_ref,
        )

        # 离散模型需要的是增量支撑力
        u_delta = np.array([
            F_left - mpc.F_eq,
            F_right - mpc.F_eq,
        ])

        # 被控对象向前运行一个周期
        x = (
            mpc.Ad @ x
            + mpc.Bd @ u_delta
        )

        if k % 20 == 0:
            print(
                f"k={k:3d}, "
                f"roll={x[0]: .6f}, "
                f"H={x[2]: .6f}, "
                f"F_L={F_left: .3f}, "
                f"F_R={F_right: .3f}"
            )

    print("最终状态：", x)
    print("目标状态：", x_ref)
if __name__ == "__main__":

    mpc = Height_Roll_MPC(
        mass=4.5,
        roll_inertia=0.1,
        half_track=0.228,
        damping=0.0,
        F_min=5.0,
        F_max=40.0,
        F_rate_max=3.0,
        equilibrium_force=None,
        Q=[
            130.0,
            1.0,
            2500.0,
            25.0,
        ],
        R=[
            0.01,
            0.01,
        ],
        sample_time=0.01,
        Horizon=15,
    )

    # F_left,F_right = (mpc.test_osqp_end_to_end())
    # print(F_left,F_right)
    test_closed_loop(mpc)