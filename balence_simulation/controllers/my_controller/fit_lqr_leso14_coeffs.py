import itertools
import math
import numpy as np
import scipy
from numpy.linalg import LinAlgError


# 10 状态、4 输入，扩张 4 个等效输入扰动。
# x = [s, ds, yaw, dyaw, theta_L, dtheta_L,
#      theta_R, dtheta_R, pitch, dpitch]
STATE_DIM = 10
# u = [T_wheel_L, T_wheel_R, Tp_L, Tp_R]
INPUT_DIM = 4
DISTURBANCE_DIM = INPUT_DIM
LESO_DIM = STATE_DIM + DISTURBANCE_DIM


# 当前模型在不同腿长下的腿质心位置和腿转动惯量。
# 每行依次为：L0 [m]、轮轴到腿质心距离 [m]、腿转动惯量 [kg*m^2]。
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
    if not LEG_PARAMETERS[0, 0] <= L0 <= LEG_PARAMETERS[-1, 0]:
        raise ValueError("腿长超出 0.16000280～0.38957695 m 的模型范围")

    l_w = np.interp(L0, LEG_PARAMETERS[:, 0], LEG_PARAMETERS[:, 1])
    I_leg = np.interp(L0, LEG_PARAMETERS[:, 0], LEG_PARAMETERS[:, 2])
    return float(l_w), float(I_leg)


def compute_AB(ll, lr, theta_lin=0.0):
    # 参数与当前 Webots 模型和 lagrange.py 保持一致。
    R_w, R_l = 0.060000003, 0.2286725
    m_w, m_l, m_b = 0.175277, 0.582519, 3.908970
    I_w, I_b, I_z = 0.0002687597, 0.008412125, 0.008735721
    g = 9.81

    c_lin = math.cos(theta_lin)
    s_lin = math.sin(theta_lin)

    l_l, l_r = ll, lr
    l_wl, I_ll = get_leg_parameters(ll)
    l_wr, I_lr = get_leg_parameters(lr)

    inv_Tr = np.matrix(
        [
            [R_w / 2, R_w / 2, 0, 0, 0],
            [
                -R_w / (2 * R_l),
                R_w / (2 * R_l),
                -l_l * c_lin / (2 * R_l),
                l_r * c_lin / (2 * R_l),
                0,
            ],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
        ]
    )

    Mat_M = np.matrix(
        [
            [
                I_w
                + I_z * R_w**2 / (4 * R_l**2)
                + R_w**2 * m_b / 4
                + R_w**2 * m_l
                + R_w**2 * m_w,
                -I_z * R_w**2 / (4 * R_l**2) + R_w**2 * m_b / 4,
                0.25 * I_z * R_w * l_l * c_lin / R_l**2
                + 0.25 * R_w * l_l * m_b * c_lin
                + R_w * l_wl * c_lin * m_l,
                -0.25 * I_z * R_w * l_r * c_lin / R_l**2
                + 0.25 * R_w * l_r * m_b * c_lin,
                0,
            ],
            [
                -I_z * R_w**2 / (4 * R_l**2) + R_w**2 * m_b / 4,
                I_w
                + I_z * R_w**2 / (4 * R_l**2)
                + R_w**2 * m_b / 4
                + R_w**2 * m_l
                + R_w**2 * m_w,
                -0.25 * I_z * R_w * l_l * c_lin / R_l**2
                + 0.25 * R_w * l_l * m_b * c_lin,
                0.25 * I_z * R_w * l_r * c_lin / R_l**2
                + 0.25 * R_w * l_r * m_b * c_lin
                + R_w * l_wr * c_lin * m_l,
                0,
            ],
            [
                0.25 * I_z * R_w * l_l * c_lin / R_l**2
                + 0.25 * R_w * l_l * m_b * c_lin
                + R_w * l_wl * c_lin * m_l,
                -0.25 * I_z * R_w * l_l * c_lin / R_l**2
                + 0.25 * R_w * l_l * m_b * c_lin,
                I_ll
                + 0.25 * I_z * l_l**2 * c_lin**2 / R_l**2
                + 0.25 * l_l**2 * m_b
                + l_wl**2 * m_l,
                -0.25 * I_z * l_l * l_r * c_lin**2 / R_l**2 + 0.25 * l_l * l_r * m_b,
                0,
            ],
            [
                -0.25 * I_z * R_w * l_r * c_lin / R_l**2
                + 0.25 * R_w * l_r * m_b * c_lin,
                0.25 * I_z * R_w * l_r * c_lin / R_l**2
                + 0.25 * R_w * l_r * m_b * c_lin
                + R_w * l_wr * c_lin * m_l,
                -0.25 * I_z * l_l * l_r * c_lin**2 / R_l**2 + 0.25 * l_l * l_r * m_b,
                I_lr
                + 0.25 * I_z * l_r**2 * c_lin**2 / R_l**2
                + 0.25 * l_r**2 * m_b
                + l_wr**2 * m_l,
                0,
            ],
            [0, 0, 0, 0, I_b],
        ]
    )

    Mat_N = np.matrix(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, -0.5 * g * l_l * m_b * c_lin - g * l_wl * c_lin * m_l, 0, 0],
            [0, 0, 0, -0.5 * g * l_r * m_b * c_lin - g * l_wr * c_lin * m_l, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    Mat_O = np.zeros((5, 5))
    Mat_O4 = np.zeros((5, 4))
    Mat_I = np.identity(5)

    Mat_E = np.matrix(
        [[1, 0, 0, 0], [0, 1, 0, 0], [-1, 0, 1, 0], [0, -1, 0, 1], [0, 0, -1, -1]]
    )

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

    MAT_A = -MAT_T * MAT_M.I * MAT_N * MAT_T.I
    MAT_B = MAT_T * MAT_M.I * MAT_E

    return MAT_A, MAT_B


def design_leso14(A, B, dt=0.003):
    """由连续 A/B 计算离散 G/H 和 14x10 观测器增益 L。"""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    # x[k+1] = G*x[k] + H*u[k]
    G, H, _, _, _ = scipy.signal.cont2discrete(
        (A, B, np.eye(STATE_DIM), np.zeros((STATE_DIM, INPUT_DIM))),
        dt,
        method="zoh",
    )

    # z=[x;d]，d[k+1]=d[k]
    Ae = np.block(  # 构造14维扩张模型
        [
            [G, H],
            [np.zeros((DISTURBANCE_DIM, STATE_DIM)), np.eye(DISTURBANCE_DIM)],
        ]
    )
    Ce = np.hstack([np.eye(STATE_DIM), np.zeros((STATE_DIM, DISTURBANCE_DIM))])

    state_poles = np.clip(np.abs(np.linalg.eigvals(G)) * 0.4, 0.2, 0.95)  # 状态估计极点，极点模乘以0.4，让观测误差收敛得更快
    observer_poles = np.concatenate([state_poles, np.full(DISTURBANCE_DIM, 0.985)])#状态估计极点和扰动估计极点0.985

    L = scipy.signal.place_poles(Ae.T, Ce.T, observer_poles).gain_matrix.T  # 求L矩阵
    L = np.asarray(np.real_if_close(L), dtype=float)
    return np.asarray(G), np.asarray(H), L


def _compute_K(A, B, q_diag, r_diag):
    """连续 LQR：u=-Kx。"""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.diag(q_diag)
    R = np.diag(r_diag)
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def _poly_features(ll, lr):
    return np.array([1.0, ll, lr, ll * ll, ll * lr, lr * lr], dtype=float)


def _fit_matrix_coeffs(samples, rows, cols):
    X = np.vstack([_poly_features(ll, lr) for ll, lr, _ in samples])
    coeffs = np.zeros((rows * cols, 6))
    for c in range(cols):
        for r in range(rows):
            y = np.array([mat[r, c] for _, _, mat in samples], dtype=float)
            beta, *_ = np.linalg.lstsq(X, y, rcond=None)
            coeffs[c * rows + r, :] = beta
    return coeffs


def fit_coefficients(
    ll_values,
    lr_values,
    q_diag,
    r_diag,
    theta_lin=0.0,
    dt=0.003,
):
    """在左右腿长网格上计算并拟合 K/G/H/L。"""
    samples_K = []
    samples_G = []
    samples_H = []
    samples_L = []

    grid_size = len(ll_values) * len(lr_values)
    print(f"Total iterations: {grid_size}")
    for ll, lr in itertools.product(ll_values, lr_values):
        A_mat, B_mat = compute_AB(ll, lr, theta_lin)
        try:
            K_mat = _compute_K(A_mat, B_mat, q_diag, r_diag)
            G_mat, H_mat, L_mat = design_leso14(A_mat, B_mat, dt)
        except (LinAlgError, ValueError, RuntimeError):
            continue
        samples_K.append((ll, lr, np.asarray(K_mat, dtype=float)))
        samples_G.append((ll, lr, G_mat))
        samples_H.append((ll, lr, H_mat))
        samples_L.append((ll, lr, L_mat))

    if not samples_K:
        raise RuntimeError("没有可用的拟合样本")

    K_coeffs = _fit_matrix_coeffs(samples_K, 4, 10)
    G_coeffs = _fit_matrix_coeffs(samples_G, 10, 10)
    H_coeffs = _fit_matrix_coeffs(samples_H, 10, 4)
    L_coeffs = _fit_matrix_coeffs(samples_L, 14, 10)

    return {
        "K_coeffs": K_coeffs,
        "G_coeffs": G_coeffs,
        "H_coeffs": H_coeffs,
        "L_coeffs": L_coeffs,
    }


def reconstruct_matrix(LL, LR, coeffs, rows, cols, eps=0.0):
    matrix = np.zeros((rows, cols), dtype=float)
    phi = _poly_features(LL, LR)
    for col in range(cols):
        for row in range(rows):
            value = float(coeffs[col * rows + row] @ phi)
            matrix[row, col] = 0.0 if abs(value) < eps else value
    return matrix


def lqr_leso_KGHL(LL, LR, coeffs):
    """按当前左右腿长恢复 K/G/H/L。"""
    K = reconstruct_matrix(LL, LR, coeffs["K_coeffs"], 4, 10, eps=1e-3)
    G = reconstruct_matrix(LL, LR, coeffs["G_coeffs"], 10, 10)
    H = reconstruct_matrix(LL, LR, coeffs["H_coeffs"], 10, 4)
    L = reconstruct_matrix(LL, LR, coeffs["L_coeffs"], 14, 10)
    return K, G, H, L


def validate_fitted_leso(coeffs, ll_values, lr_values):
    """检查拟合后的观测器在腿长范围内是否稳定。"""
    Ce = np.hstack([np.eye(STATE_DIM), np.zeros((STATE_DIM, DISTURBANCE_DIM))])
    worst_radius = -np.inf
    worst_leg_length = None

    for ll, lr in itertools.product(ll_values, lr_values):
        _, G_fit, H_fit, L_fit = lqr_leso_KGHL(ll, lr, coeffs)
        Ae_fit = np.block(
            [
                [G_fit, H_fit],
                [np.zeros((DISTURBANCE_DIM, STATE_DIM)), np.eye(DISTURBANCE_DIM)],
            ]
        )
        radius = float(np.max(np.abs(np.linalg.eigvals(Ae_fit - L_fit @ Ce))))
        if radius > worst_radius:
            worst_radius = radius
            worst_leg_length = (float(ll), float(lr))

    if worst_radius >= 1.0:
        raise RuntimeError(f"LESO 拟合后不稳定: rho={worst_radius:.9f}")
    return worst_radius, worst_leg_length


if __name__ == "__main__":
    ll_grid = np.linspace(0.16000280, 0.38957695, 24)
    lr_grid = np.linspace(0.16000280, 0.38957695, 24)
    theta_lin = 0.0
    dt = 0.003

    q_diag = [1000, 100, 1000, 100, 12000, 50, 12000, 50, 5000, 40]
    r_diag = [60, 60, 1.5, 1.5]

    coeffs = fit_coefficients(ll_grid, lr_grid, q_diag, r_diag, theta_lin, dt)

    max_rho, worst_length = validate_fitted_leso(
        coeffs,
        np.linspace(0.16000280, 0.38957695, 17),
        np.linspace(0.16000280, 0.38957695, 17),
    )
    print(f"Fitted LESO max rho: {max_rho:.9f}, LL/LR={worst_length}")

    output_lines = []

    def format_array_decl(name, arr, eps=1e-3):
        rows, cols = arr.shape
        output_lines.append(f"float {name}[{rows}][{cols}] = {{")
        for i, row in enumerate(arr):
            filtered = [0.0 if abs(v) < eps else v for v in row]
            row_txt = ", ".join(f"{v:.10g}" for v in filtered)
            if i == rows - 1:
                output_lines.append(f"        {{{row_txt}}}}};")
            else:
                output_lines.append(f"        {{{row_txt}}},")

    format_array_decl("K_out", coeffs["K_coeffs"])
    output_lines.append("")

    format_array_decl("G_out", coeffs["G_coeffs"], eps=1e-12)
    output_lines.append("")
    format_array_decl("H_out", coeffs["H_coeffs"], eps=1e-12)
    output_lines.append("")
    format_array_decl("L_out", coeffs["L_coeffs"], eps=1e-12)

    full_output = "\n".join(output_lines)

    import subprocess

    try:
        subprocess.run(
            ["clip"],
            input=full_output,
            text=True,
            check=True,
        )
        print("Output copied to clipboard successfully.")
    except Exception as e:
        print(f"Could not copy to clipboard: {e}")
        print(full_output)
