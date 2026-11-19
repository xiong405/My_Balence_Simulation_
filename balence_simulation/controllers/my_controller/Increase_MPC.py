import numpy as np
import scipy.sparse as sparse
from scipy.signal import cont2discrete
import osqp
import matplotlib.pyplot as plt
import time
class Height_Roll_MPC_inc:
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
        # 7.构建增广矩阵
        # Delta_x(k+1) = Ad * Delta_x(k) + Bd * Delta_u(k)
        #       x(k+1) = x(k) + Delta_x(k+1)  -> x(k+1) = x(k) + Ad * Delta_x(k) + Bd * Delta_u(k)
        self.Ad_inc = np.block([[self.Ad,np.zeros((self.nx,self.nx))],#block分块矩阵组装
                                [self.Ad,np.eye(self.nx)]])#(8,8)
        self.Bd_inc = np.vstack((self.Bd,self.Bd))#vstack纵向堆叠  # 8×2
        self.C_inc = np.block([np.zeros((self.nx,self.nx)),np.eye(self.nx)])#hstack横向拼接  # 4×8

        #8.增量预测矩阵 
        #U = E_previous * u_previous + S_delta * Delta_U
        self.E_previous = np.kron( np.ones((self.N,1)), np.eye(self.nu))#(30,2)
        self.S_delta = np.kron(np.tril(np.ones((self.N,self.N))),np.eye(self.nu))#(30,30)



        #获得M,C矩阵
        self.M_inc,self.C_delta = self.MPC_Prediction_M_C_inc()
        #获取Q_ba,R_ba,H_qp
        self.Q_ba, self.R_ba, self.H_qp = (self.MPC_Prediction_Q_R_H_qp())



        self.u_previous = np.zeros(self.nu,dtype=float) # 初始化u_previous
        self.x_previous = None#初始化上一次状态，用于计算Delta_x
        self.F_constraint = (self.MPC_Constraint_Matrix())# 完整约束矩阵 (60,30)
        (self.lower_constraint,self.upper_constraint) = self.MPC_Force_Constraint()
        self.MPC_Init_OSQP()# 初始化OSQP


########################构造预测15步的状态矩阵和代价函数J###################
    #构造M_inc和C_delta矩阵
    def MPC_Prediction_M_C_inc(self):
        # 构造MPC预测矩阵：X = M_inc @ z(k) + C_delta @ Delta_U
        #z(k) = [Delta_x(k); x(k)]
        M_inc = np.zeros((self.N * self.nx, 2*self.nx))  # (60, 8)
        C_delta = np.zeros((self.N * self.nx,self.N * self.nu)) #*(60,30)
        for i in range(self.N):
            row_start = i * self.nx#构造行
            row_end = (i + 1) * self.nx
            M_inc[row_start:row_end,:] = self.C_inc @ np.linalg.matrix_power(self.Ad_inc, i+1)
            for j in range (i + 1):
                col_start = j * self.nu#构造列
                col_end = (j + 1) * self.nu
                C_delta[row_start:row_end,col_start:col_end] = self.C_inc @ np.linalg.matrix_power(self.Ad_inc, i - j) @ self.Bd_inc
        return M_inc ,C_delta


    #构造Q,R,H_qp矩阵
    def MPC_Prediction_Q_R_H_qp(self):
        Q_ba = np.kron(np.eye(self.N),self.Q)#(60,60)
        R_ba = np.kron(np.eye(self.N),self.R)#(30,30)
        H_qp = 2.0 * (self.C_delta.T @ Q_ba @ self.C_delta + R_ba)#(30,30)
        H_qp = 0.5 * (H_qp + H_qp.T)# 消除浮点计算造成的微小不对称
        return Q_ba, R_ba, H_qp


    #求解f_QP
    def MPC_Calculate_f_qp(self, x, x_ref):
        """f_QP = 2C_T @ Q @ (M @ x - Xref)"""
        x = np.asarray(x,dtype=float).reshape(self.nx)# 当前状态：(4,)通过reshape转化为1维度
        x_ref = np.asarray(x_ref,dtype=float).reshape(self.nx)# 单步参考状态：(4,)
        if self.x_previous is None:
            delta_x = np.zeros(self.nx,dtype=float)#第一次令Delta_x=0
        else:
            delta_x = x - self.x_previous#Delta_x(k)=x(k)-x(k-1)
        z = np.concatenate((delta_x,x))#z(k)=[Delta_x(k);x(k)]，(8,)
        X_ref = np.tile(x_ref,self.N)# 未来N步参考状态：(60,)
        X_free = self.M_inc @ z#未来15步自由预测状态(60,)
        f_qp = 2.0 * (self.C_delta.T @ self.Q_ba @ (X_free - X_ref))#(30,)
        self.x_previous = x.copy()#保存当前状态，下一周期使用
        return np.asarray(f_qp,dtype=float,).reshape(-1)#最后输出一维度的



#######################构建力矩约束矩阵F_constraint 和约束力lower_constraint,upper_constraint#############
# [ U_min - E_previous @ u_previous ]     [ S_delta ]
# [                                  ] <= [         ] @ Delta_U
# [          -Delta_U_max            ]     [    I    ]
#
#                         [ U_max - E_previous @ u_previous ]
#                      <= [                                    ]
#                         [            Delta_U_max             ]

    #构造完整约束矩阵F_constraint
    def MPC_Constraint_Matrix(self):
        """
        OSQP决策变量为Delta_U
        """
        nU = self.N * self.nu#(30)
        F_limit = self.S_delta#实际控制量约束(30,30)
        F_rate = np.eye(nU,dtype=float)#控制增量约束(30,30)
        F_constraint = np.vstack((F_limit,F_rate))#(60,30)
        return sparse.csc_matrix(F_constraint)



    #力矩约束函数,生成l,u(60,)用于约束的最大值和最小值
    def MPC_Force_Constraint(self): 
        nU = self.N * self.nu#(30)
        delta_F_min = self.F_min - self.F_eq
        delta_F_max = self.F_max - self.F_eq
        #delta力矩
        U_previous = self.E_previous @ self.u_previous#(30,) 将上一次控制量复制到未来15步
        lower_force = np.full(nU,delta_F_min,dtype=float) - U_previous#新建一个数组，全部元素都填同一个值delta_F_min
        upper_force = np.full(nU,delta_F_max,dtype=float) - U_previous

        #变化率
        lower_rate = np.full(nU,-self.F_rate_max,dtype=float)#(30,)
        upper_rate = np.full(nU,self.F_rate_max,dtype=float)
        #完整约束上下界
        lower_constraint = np.concatenate((lower_force,lower_rate))#(60,)
        upper_constraint = np.concatenate((upper_force,upper_rate))
        return (lower_constraint,upper_constraint)


###############构造QP求解器#################

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

        Delta_U_out = np.asarray(result.x,dtype=float).reshape(-1)#未来15步控制增量(30,)
        delta_u_now = Delta_U_out[:self.nu]#只取第一步控制增量(2,)
        u_now = self.u_previous + delta_u_now#u(k)=u(k-1)+Delta_u(k)
        self.u_previous = u_now.copy()#保存当前相对支撑力
        F_left = (self.F_eq + u_now[0])
        F_right = (self.F_eq + u_now[1])
        return (float(F_left),float(F_right),)

# if __name__ == "__main__":

#     # 只初始化一次
#     mpc = Height_Roll_MPC_inc(
#         mass=4.5,
#         roll_inertia=0.1,
#         half_track=0.228,
#         damping=0.0,
#         F_min=5.0,
#         F_max=40.0,
#         F_rate_max=3.0,
#         equilibrium_force=None,
#         Q=[
#             130.0,
#             1.0,
#             2500.0,
#             25.0,
#         ],
#         R=[
#             0.01,
#             0.01,
#         ],
#         sample_time=0.01,
#         Horizon=15,
#     )

#     while True:

#         #获取当前状态
#         roll, roll_dot, H, H_dot, target_H = get_date()

#         #当前状态
#         x = np.array(
#             [
#                 roll,
#                 roll_dot,
#                 H,
#                 H_dot,
#             ],
#             dtype=float,
#         )

#         # 目标状态
#         x_ref = np.array(
#             [
#                 0.0,        # roll_ref
#                 0.0,        # roll_dot_ref
#                 target_H,   # H_ref
#                 0.0,        # H_dot_ref
#             ],
#             dtype=float,
#         )

#         # 计算本周期左右腿实际支撑力
#         F_left, F_right = mpc.solve_OSQP(
#             x,
#             x_ref,
#         )

#         # 发送回C板
#         send_date(
#             F_left,
#             F_right,
#         )


