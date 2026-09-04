# -*- coding: utf-8 -*-
# @Time : 2024/9/12 17:20
# @Author : wangfaqi
# @File ： lagrange.py

import numpy as np
import scipy


# 腿长 L0 [m]、轮轴到腿质心距离 l_w [m]、腿转动惯量 I_leg [kg*m^2]
LEG_PARAMETERS = np.array(
    [
        [0.16000280, 0.15746146, 0.02653100],
        [0.16998440, 0.16246321, 0.02690439],
        [0.17996804, 0.16760882, 0.02730323],
        [0.18995341, 0.17288249, 0.02772755],
        [0.19994023, 0.17827176, 0.02817742],
        [0.20992831, 0.18376558, 0.02865290],
        [0.21991748, 0.18935411, 0.02915406],
        [0.22990758, 0.19502861, 0.02968099],
        [0.23989851, 0.20078128, 0.03023378],
        [0.24989017, 0.20660510, 0.03081251],
        [0.25988247, 0.21249380, 0.03141730],
        [0.26987533, 0.21844173, 0.03204827],
        [0.27986871, 0.22444379, 0.03270554],
        [0.28986255, 0.23049535, 0.03338925],
        [0.29985679, 0.23659224, 0.03409956],
        [0.30985141, 0.24273061, 0.03483666],
        [0.31984636, 0.24890697, 0.03560074],
        [0.32984162, 0.25511810, 0.03639204],
        [0.33983716, 0.26136098, 0.03721082],
        [0.34983295, 0.26763282, 0.03805741],
        [0.35982898, 0.27393095, 0.03893218],
        [0.36982522, 0.28025285, 0.03983560],
        [0.37982166, 0.28659595, 0.04076824],
        [0.38957695, 0.29095577, 0.04142493],
    ]
)


def get_leg_parameters(L0):
    """按当前腿长返回 (轮轴到腿质心距离, 腿转动惯量)。"""
    if not LEG_PARAMETERS[0, 0] <= L0 <= LEG_PARAMETERS[-1, 0]:
        raise ValueError("腿长超出查表范围 0.16000280～0.38957695 m")

    l_w = np.interp(L0, LEG_PARAMETERS[:, 0], LEG_PARAMETERS[:, 1])
    I_leg = np.interp(L0, LEG_PARAMETERS[:, 0], LEG_PARAMETERS[:, 2])
    return float(l_w), float(I_leg)


def K(ll, lr, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4):
    # 赋值
    R_w, R_l, m_w, m_l, m_b, I_w, I_b, I_z, g = (
        0.060000003,
        0.2286725,
        0.175277,
        0.582519,
        3.908970,
        0.0002687597,
        0.008412125,
        0.008735721,
        9.81,
    )
    # R_w, R_l, l_c, m_w, m_l, m_b, I_w, I_b, I_z, g = 0.1, 0.2265, 0, 1.4642, 0.88, 4.396, 0.0058597, 0.0087935, 0.1525, 9.8
    l_l, l_r = ll, lr
    l_wl, I_ll = get_leg_parameters(ll)
    l_wr, I_lr = get_leg_parameters(lr)

    # 被注释部分为采用拉式方程建立机器人状态方程的符号运算过程

    # t = symbols('t')
    #
    # theta_wl = Function('theta_wl')
    # theta_wr = Function('theta_wr')
    # theta_ll = Function('theta_ll')
    # theta_lr = Function('theta_lr')
    # theta_b = Function('theta_b')
    #
    # R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g \
    #     = symbols('R_w,R_l,l_l,l_r,l_wl,l_wr,l_bl,l_br,l_c,m_w,m_l,m_b,I_w,I_ll,I_lr,I_b,I_z,g')
    #
    # s = R_w / 2 * (theta_wl(t) + theta_wr(t))
    # h_b = l_l * cos(theta_ll(t)) + l_r * cos(theta_lr(t))
    # s_ll = R_w * theta_wl(t) + l_wl * sin(theta_ll(t))
    # s_lr = R_w * theta_wr(t) + l_wr * sin(theta_lr(t))
    # h_ll = h_b - l_bl * cos(theta_ll(t))
    # h_lr = h_b - l_br * cos(theta_lr(t))
    #
    # # Equations = [Eq(R_w*theta_wl(t)-s_b+R_l*fai+l_l*sin(theta_ll(t)),0),
    # #              Eq(R_w*theta_wr(t)-s_b-R_l*fai+l_r*sin(theta_lr(t)),0)]
    # #
    # # solutions =solve(Equations,(fai,s_b))
    # # print(solutions)
    #
    # fai = -R_w * theta_wl(t) / (2 * R_l) + R_w * theta_wr(t) / (2 * R_l) - l_l * sin(theta_ll(t)) / (
    #         2 * R_l) + l_r * sin(
    #     theta_lr(t)) / (2 * R_l)
    # s_b = R_w * theta_wl(t) / 2 + R_w * theta_wr(t) / 2 + l_l * sin(theta_ll(t)) / 2 + l_r * sin(theta_lr(t)) / 2
    #
    # dot_s = diff(s, t)
    # ddot_s = diff(dot_s, t)
    #
    # dot_fai = diff(fai, t)
    # ddot_fai = diff(dot_fai, t)
    #
    # dot_s_b = diff(s_b, t)
    # ddot_s_b = diff(dot_s_b, t)
    #
    # dot_h_b = diff(h_b, t)
    # ddot_h_b = diff(dot_h_b, t)
    #
    # dot_s_ll = diff(s_ll, t)
    # dot_s_lr = diff(s_lr, t)
    # ddot_s_ll = diff(dot_s_ll, t)
    # ddot_s_lr = diff(dot_s_lr, t)
    #
    # dot_h_ll = diff(h_ll, t)
    # dot_h_lr = diff(h_lr, t)
    # ddot_h_ll = diff(dot_h_ll, t)
    # ddot_h_lr = diff(dot_h_lr, t)
    #
    # # linearization
    # # dot_s = R_w * (Derivative(theta_wl(t), t) + Derivative(theta_wr(t), t)) / 2
    # # ddot_s = R_w * (Derivative(theta_wl(t), (t, 2)) + Derivative(theta_wr(t), (t, 2))) / 2
    # #
    # # dot_fai = -R_w * Derivative(theta_wl(t), t) / (2 * R_l) + R_w * Derivative(theta_wr(t), t) / (
    # #         2 * R_l) - l_l * Derivative(theta_ll(t), t) / (2 * R_l) + l_r * Derivative(theta_lr(t), t) / (2 * R_l)
    # # ddot_fai = -R_w * Derivative(theta_wl(t), (t, 2)) / (2 * R_l) + R_w * Derivative(theta_wr(t), (t, 2)) / (
    # #         2 * R_l) - l_l * Derivative(theta_ll(t), (t, 2)) / (2 * R_l) + l_r * Derivative(theta_lr(t), (t, 2)) / (
    # #                    2 * R_l)
    # #
    # # dot_s_b = R_w * Derivative(theta_wl(t), t) / 2 + R_w * Derivative(theta_wr(t), t) / 2 + l_l * Derivative(
    # #     theta_ll(t),
    # #     t) / 2 + l_r * Derivative(
    # #     theta_lr(t), t) / 2
    # # ddot_s_b = R_w * Derivative(theta_wl(t), (t, 2)) / 2 + R_w * Derivative(theta_wr(t), (t, 2)) / 2 + l_l * Derivative(
    # #     theta_ll(t), (t, 2)) / 2 + l_r * Derivative(theta_lr(t), (t, 2)) / 2
    # #
    # # dot_h_b = 0
    # # ddot_h_b = 0
    # #
    # # dot_s_ll = R_w * Derivative(theta_wl(t), t) + l_wl * Derivative(theta_ll(t), t)
    # # dot_s_lr = R_w * Derivative(theta_wr(t), t) + l_wr * Derivative(theta_lr(t), t)
    # # ddot_s_ll = R_w * Derivative(theta_wl(t), (t, 2)) + l_wl * Derivative(theta_ll(t), (t, 2))
    # # ddot_s_lr = R_w * Derivative(theta_wr(t), (t, 2)) + l_wr * Derivative(theta_lr(t), (t, 2))
    # #
    # # dot_h_ll = 0
    # # dot_h_lr = 0
    # # ddot_h_ll = 0
    # # ddot_h_lr = 0
    #
    # T_w = m_w*R_w*R_w*(Derivative(theta_wl(t), t)**2+Derivative(theta_wr(t), t)**2)/2
    # T_1 = m_l*(dot_s_ll**2+dot_h_ll**2)/2+m_l*(dot_s_lr**2+dot_h_lr**2)/2
    # T_2 = m_b*(dot_s_b**2+dot_h_b**2)/2
    # Ro_w = I_w*(Derivative(theta_wl(t), t)**2+Derivative(theta_wr(t), t)**2)/2
    # Ro_1 = (I_ll*Derivative(theta_ll(t),t)**2 + I_lr*Derivative(theta_lr(t),t)**2)/2
    # Ro_2 = (I_b*Derivative(theta_b(t),t)**2)/2
    # Ro_3 = (I_z*dot_fai**2)/2
    # U = m_l*g*l_wl*cos(theta_ll(t))+m_l*g*l_wr*cos(theta_lr(t))+m_b*g*(l_l*cos(theta_ll(t))/2+l_r*cos(theta_lr(t))/2)
    # L = T_w+T_1+T_2+Ro_w+Ro_1+Ro_2+Ro_3-U
    #
    # pL_pdtheta_wl = diff(L,diff(theta_wl(t),t))
    # pL_pdtheta_wr = diff(L,diff(theta_wr(t),t))
    # pL_pdtheta_ll = diff(L,diff(theta_ll(t),t))
    # pL_pdtheta_lr = diff(L,diff(theta_lr(t),t))
    # pL_pdtheta_b = diff(L,diff(theta_b(t),t))
    #
    #
    # pL_ptheta_wl = diff(L,theta_wl(t))
    # pL_ptheta_wr = diff(L,theta_wr(t))
    # pL_ptheta_ll = diff(L,theta_ll(t))
    # pL_ptheta_lr = diff(L,theta_lr(t))
    # pL_ptheta_b = diff(L,theta_b(t))
    #
    # ddt_pL_pdtheta_wl = diff(pL_pdtheta_wl,t)
    # ddt_pL_pdtheta_wr = diff(pL_pdtheta_wr,t)
    # ddt_pL_pdtheta_ll = diff(pL_pdtheta_ll,t)
    # ddt_pL_pdtheta_lr = diff(pL_pdtheta_lr,t)
    # ddt_pL_pdtheta_b = diff(pL_pdtheta_b,t)
    #
    # LARGE = [ddt_pL_pdtheta_wl - pL_ptheta_wl,
    #          ddt_pL_pdtheta_wr - pL_ptheta_wr,
    #          ddt_pL_pdtheta_ll - pL_ptheta_ll,
    #          ddt_pL_pdtheta_lr - pL_ptheta_lr,
    #          ddt_pL_pdtheta_b - pL_ptheta_b]
    #
    #
    # M = Matrix([[0,0,0,0,0],
    #             [0,0,0,0,0],
    #             [0,0,0,0,0],
    #             [0,0,0,0,0],
    #             [0,0,0,0,0]])
    #
    # for i in range(5):
    #     M[5*i+0] = diff(LARGE[i],diff(diff(theta_wl(t))))
    #     M[5*i+1] = diff(LARGE[i],diff(diff(theta_wr(t))))
    #     M[5*i+2] = diff(LARGE[i],diff(diff(theta_ll(t))))
    #     M[5*i+3] = diff(LARGE[i],diff(diff(theta_lr(t))))
    #     M[5*i+4] = diff(LARGE[i],diff(diff(theta_b(t))))
    #
    # N = Matrix([[0],
    #             [0],
    #             [0],
    #             [0],
    #             [0]])
    #
    # for i in range(5):
    #     N[i] = LARGE[i] - M[5*i+0]*diff(diff(theta_wl(t))) - M[5*i+1]*diff(diff(theta_wr(t))) - M[5*i+2]*diff(diff(theta_ll(t))) \
    #          - M[5*i+3]*diff(diff(theta_lr(t))) - M[5*i+4]*diff(diff(theta_b(t)))
    #     print(latex(N[i]))
    #
    # print(latex(M))
    #
    # E = Matrix([[1,0,0,0],
    #             [0,1,0,0],
    #             [-1,0,-1,0],
    #             [0,-1,0,-1],
    #             [0,0,1,1]])
    #
    inv_Tr = np.matrix(
        [
            [R_w / 2, R_w / 2, 0, 0, 0],
            [-R_w / (2 * R_l), R_w / (2 * R_l), -l_l / (2 * R_l), l_r / (2 * R_l), 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
        ]
    )
    #
    #
    # def MatLinear(A):
    #     return eval(str(A).replace('Derivative(theta_wl(t), t)**2','0').replace('Derivative(theta_wr(t), t)**2','0').replace('Derivative(theta_ll(t), t)**2','0').replace('Derivative(theta_lr(t), t)**2','0')
    #                 .replace('Derivative(theta_wl(t), (t, 2))', '0').replace('Derivative(theta_wr(t), (t, 2))', '0').replace('Derivative(theta_ll(t), (t, 2))', '0').replace('Derivative(theta_lr(t), (t, 2))', '0')
    #                 .replace('cos(theta_ll(t))','1').replace('cos(theta_lr(t))','1').replace('sin(theta_ll(t))','theta_ll(t)').replace('sin(theta_lr(t))','theta_lr(t)')
    #                 .replace('theta_ll(t)**2','0').replace('theta_lr(t)**2','0')
    #                 .replace('l_br*theta_lr(t) - l_r*theta_lr(t)','0').replace('l_bl*theta_ll(t) - l_l*theta_ll(t)','0')
    #                 .replace('2*l_br*theta_lr(t) - 2*l_r*theta_lr(t)', '0').replace('2*l_bl*theta_ll(t) - 2*l_l*theta_ll(t)', '0')
    #                 .replace('theta_ll(t)*theta_lr(t)', '0'))
    #
    #
    # print((latex(MatLinear(M))))
    # print(MatLinear(M))
    # print(MatLinear(N[0]))
    # print(MatLinear(N[1]))
    # print(MatLinear(N[2]))
    # print(MatLinear(N[3]))
    # print(MatLinear(N[4]))

    Mat_M = np.matrix(
        [
            [
                I_w
                + I_z * R_w**2 / (4 * R_l**2)
                + R_w**2 * m_b / 4
                + R_w**2 * m_l
                + R_w**2 * m_w,
                -I_z * R_w**2 / (4 * R_l**2) + R_w**2 * m_b / 4,
                I_z * R_w * l_l / (4 * R_l**2) + R_w * l_l * m_b / 4 + R_w * l_wl * m_l,
                -I_z * R_w * l_r / (4 * R_l**2) + R_w * l_r * m_b / 4,
                0,
            ],
            [
                -I_z * R_w**2 / (4 * R_l**2) + R_w**2 * m_b / 4,
                I_w
                + I_z * R_w**2 / (4 * R_l**2)
                + R_w**2 * m_b / 4
                + R_w**2 * m_l
                + R_w**2 * m_w,
                -I_z * R_w * l_l / (4 * R_l**2) + R_w * l_l * m_b / 4,
                I_z * R_w * l_r / (4 * R_l**2) + R_w * l_r * m_b / 4 + R_w * l_wr * m_l,
                0,
            ],
            [
                I_z * R_w * l_l / (4 * R_l**2) + R_w * l_l * m_b / 4 + R_w * l_wl * m_l,
                -I_z * R_w * l_l / (4 * R_l**2) + R_w * l_l * m_b / 4,
                I_ll + I_z * l_l**2 / (4 * R_l**2) + l_l**2 * m_b / 4 + l_wl**2 * m_l,
                -I_z * l_l * l_r / (4 * R_l**2) + l_l * l_r * m_b / 4,
                0,
            ],
            [
                -I_z * R_w * l_r / (4 * R_l**2) + R_w * l_r * m_b / 4,
                I_z * R_w * l_r / (4 * R_l**2) + R_w * l_r * m_b / 4 + R_w * l_wr * m_l,
                -I_z * l_l * l_r / (4 * R_l**2) + l_l * l_r * m_b / 4,
                I_lr + I_z * l_r**2 / (4 * R_l**2) + l_r**2 * m_b / 4 + l_wr**2 * m_l,
                0,
            ],
            [0, 0, 0, 0, I_b],
        ]
    )

    Mat_N = np.matrix(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, -g * l_l * m_b / 2 - g * l_wl * m_l, 0, 0],
            [0, 0, 0, -g * l_r * m_b / 2 - g * l_wr * m_l, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    Mat_O = np.zeros((5, 5))
    Mat_O4 = np.zeros((5, 4))
    Mat_I = np.identity(5)

    # Mat_E的正负号需要根据自己所建立lagrange方程的符号做对应的调整
    Mat_E = np.matrix(
        [[1, 0, 0, 0], [0, 1, 0, 0], [-1, 0, 1, 0], [0, -1, 0, 1], [0, 0, -1, -1]]
    )

    # print(Mat_O, Mat_I)

    MAT_M = np.matrix(np.block([[Mat_I, Mat_O], [Mat_O, Mat_M]]))
    MAT_N = np.matrix(np.block([[Mat_O, -Mat_I], [Mat_N, Mat_O]]))
    MAT_E = np.matrix(np.block([[Mat_O4], [Mat_E]]))
    MAT_T1 = np.matrix(np.block([[inv_Tr, Mat_O], [Mat_O, inv_Tr]]))
    MAT_T2 = np.matrix(
        [
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
    )
    MAT_T = MAT_T2 * MAT_T1

    # print(MAT_M)
    # print(MAT_E)

    # Mat_M * ddot_q + Mat_N * q = MAT_E * tao
    # MAT_M * MAT_T.I * dot_Q + MAT_N * MAT_T.I * Q = MAT_E * tao

    ## 在这里补充一下q和Q的含义
    ## q = np.matrix([[theta_wl],[theta_wr],[theta_ll],[theta_lr],[theta_b]])
    ## Q = np.matrix([[s],[dot_s],[fai],[dot_fai],[theta_ll],[dot_theta_ll],[theta_lr],[dot_theta_lr],[theta_b],[dot_theta_b]])
    ## 从q到Q进行了坐标变换，两个轮的角度theta_wl和theta_wr变换为机器人整体位移s和航向角fai，MAT_T就是变换矩阵，同时需要注意维度变化，由5维q变为10维Q。

    # MAT_M * MAT_T.I * dot_Q = - MAT_N * MAT_T.I * Q + MAT_E * tao
    # dot_Q = - (MAT_M * MAT_T.I).I * MAT_N * MAT_T.I * Q + (MAT_M * MAT_T.I).I * MAT_E * tao
    # dot_Q = - MAT_T * MAT_M.I * MAT_N * MAT_T.I * Q + MAT_T * MAT_M.I * MAT_E * tao

    MAT_A = -MAT_T * MAT_M.I * MAT_N * MAT_T.I
    MAT_B = MAT_T * MAT_M.I * MAT_E

    # print(MAT_A)
    # print(MAT_B)

    QQ = np.zeros((10, 10))

    QQ[0][0] = q1
    QQ[1][1] = q2
    QQ[2][2] = q3
    QQ[3][3] = q4
    QQ[4][4] = q5
    QQ[5][5] = q6
    QQ[6][6] = q7
    QQ[7][7] = q8
    QQ[8][8] = q9
    QQ[9][9] = q10

    R = np.matrix([[r1, 0, 0, 0], [0, r2, 0, 0], [0, 0, r3, 0], [0, 0, 0, r4]])

    # 连续
    P = scipy.linalg.solve_continuous_are(MAT_A, MAT_B, QQ, R)
    K = R.I * MAT_B.T * P

    # 当然也可以使用离散LQR计算K值，在仿真里都差不多，这里附上有关代码
    # def K_discrete_lqr(A, B, Q, R):
    #     C = np.zeros((4, 10))
    #     D = np.zeros((4, 4))
    #     system_LIP = control.ss(A, B, C, D)
    #     system_dLIP = control.c2d(system_LIP, 0.01)
    #     # print(system_dLIP)
    #     MAT_dA = np.matrix(system_dLIP.A)
    #     MAT_dB = np.matrix(system_dLIP.B)
    #     P = scipy.linalg.solve_discrete_are(MAT_dA, MAT_dB, Q, R)
    #     K = (R + MAT_dB.T * P * MAT_dB).I * MAT_dB.T * P * MAT_dA
    #     return K

    return K


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=300)

    # # print(A,B,Q,R)
    # a = time.time()
    # k1 = np.matrix(K_continuous_lqr(A, B, Q, R))
    k1 = K(0.2, 0.2, 10, 5, 50, 1, 50, 1, 50, 1, 50, 1, 1, 1, 1, 1)
    print(k1)

    for i in range(4):
        for j in range(10):
            print("K[{}] = {};".format(10 * i + j, round(k1[i, j], 8)))
            # print(round(k1[i,j],8))
