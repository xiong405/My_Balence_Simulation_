import itertools
import math
from pathlib import Path
import numpy as np
import scipy
from numpy.linalg import LinAlgError


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


def _compute_K(
    ll,
    lr,
    q1,
    q2,
    q3,
    q4,
    q5,
    q6,
    q7,
    q8,
    q9,
    q10,
    r1,
    r2,
    r3,
    r4,
    use_discrete=False,
    dt=0.002,
    theta_lin=0.0,
):

    MAT_A, MAT_B = compute_AB(ll, lr, theta_lin)

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

    if use_discrete:

        def K_discrete_lqr(A, B, Q, R, dt):
            C = np.zeros((4, 10))
            D = np.zeros((4, 4))
            MAT_dA, MAT_dB, _, _, _ = scipy.signal.cont2discrete(
                (np.asarray(A), np.asarray(B), C, D), dt
            )
            MAT_dA = np.matrix(MAT_dA)
            MAT_dB = np.matrix(MAT_dB)
            P = scipy.linalg.solve_discrete_are(MAT_dA, MAT_dB, Q, R)
            K = (R + MAT_dB.T * P * MAT_dB).I * MAT_dB.T * P * MAT_dA
            return K

        K_gain = K_discrete_lqr(MAT_A, MAT_B, QQ, R, dt)
    else:
        P = scipy.linalg.solve_continuous_are(MAT_A, MAT_B, QQ, R)
        K_gain = R.I * MAT_B.T * P

    return K_gain


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


def fit_coefficients(ll_values, lr_values, q_diag, r_diag, theta_lin=0.0, dt=0.002):
    samples_K = []
    samples_A = []
    samples_B = []

    q1, q2, q3, q4, q5, q6, q7, q8, q9, q10 = q_diag
    r1, r2, r3, r4 = r_diag

    grid_size = len(ll_values) * len(lr_values)
    print(f"Total iterations: {grid_size}. This might take a few seconds...")
    for ll, lr in itertools.product(ll_values, lr_values):
        A_mat, B_mat = compute_AB(ll, lr, theta_lin)
        try:
            K_mat = _compute_K(
                ll,
                lr,
                q1,
                q2,
                q3,
                q4,
                q5,
                q6,
                q7,
                q8,
                q9,
                q10,
                r1,
                r2,
                r3,
                r4,
                theta_lin=theta_lin,
                dt=dt,
            )
        except LinAlgError:
            continue
        samples_A.append((ll, lr, np.asarray(A_mat, dtype=float)))
        samples_B.append((ll, lr, np.asarray(B_mat, dtype=float)))
        samples_K.append((ll, lr, np.asarray(K_mat, dtype=float)))

    if not samples_K:
        raise RuntimeError("No stable samples available; adjust Q/R or sampling grid.")

    K_coeffs = _fit_matrix_coeffs(samples_K, 4, 10)
    A_coeffs = _fit_matrix_coeffs(samples_A, 10, 10)
    B_coeffs = _fit_matrix_coeffs(samples_B, 10, 4)

    return {
        "K_coeffs": K_coeffs,
        "A_coeffs": A_coeffs,
        "B_coeffs": B_coeffs,
    }


def _format_cpp_array(arr, eps=1e-3):
    lines = []
    for row in arr:
        filtered = [0.0 if abs(v) < eps else v for v in row]
        row_txt = ", ".join(f"{v:.10g}" for v in filtered)
        lines.append(f"    {{{row_txt}}}")
    return "{\n" + ",\n".join(lines) + "\n}"


def export_cpp_library(
    coeffs,
    header_path="lqr_coeffs.h",
    source_path="lqr_coeffs.cpp",
    namespace="lqr_fit",
):

    header_path = Path(header_path)
    source_path = Path(source_path)

    K_coeffs = np.asarray(coeffs["K_coeffs"], dtype=float)
    A_coeffs = np.asarray(coeffs["A_coeffs"], dtype=float)
    B_coeffs = np.asarray(coeffs["B_coeffs"], dtype=float)

    header = f"""#pragma once
#include <cstddef>

namespace {namespace} {{

extern const double K_out[40][6];
extern const double A_out[100][6];
extern const double B_out[40][6];

void lqr_KAB(double LL, double LR,
             double K_matrix[4][10],
             double A_matrix[10][10],
             double B_matrix[10][4]);

}}

"""

    source = f"""#include <array>
#include "{header_path.name}"

namespace {namespace} {{

const double K_out[40][6] = {_format_cpp_array(K_coeffs)};
const double A_out[100][6] = {_format_cpp_array(A_coeffs)};
const double B_out[40][6] = {_format_cpp_array(B_coeffs)};

void lqr_KAB(double LL, double LR,
             double K_matrix[4][10],
             double A_matrix[10][10],
             double B_matrix[10][4]) {{
    double phi[6] = {{1.0, LL, LR, LL * LL, LL * LR, LR * LR}};
    const double eps = 1e-3;

    for (std::size_t col = 0; col < 10; ++col) {{
        for (std::size_t row = 0; row < 4; ++row) {{
            const double* coeffs = K_out[col * 4 + row];
            double val = 0.0;
            for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
            K_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
        }}
    }}

    for (std::size_t col = 0; col < 10; ++col) {{
        for (std::size_t row = 0; row < 10; ++row) {{
            const double* coeffs = A_out[col * 10 + row];
            double val = 0.0;
            for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
            A_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
        }}
    }}

    for (std::size_t col = 0; col < 4; ++col) {{
        for (std::size_t row = 0; row < 10; ++row) {{
            const double* coeffs = B_out[col * 10 + row];
            double val = 0.0;
            for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
            B_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
        }}
    }}
}}

}}

"""

    header_path.write_text(header, encoding="utf-8")
    source_path.write_text(source, encoding="utf-8")
    return header_path, source_path


def lqr_KAB(LL, LR, K_coeffs, A_coeffs, B_coeffs):
    K_matrix = np.zeros((4, 10))
    A_matrix = np.zeros((10, 10))
    B_matrix = np.zeros((10, 4))
    phi = _poly_features(LL, LR)
    eps = 1e-3

    for col in range(10):
        for row in range(4):
            coeffs = K_coeffs[col * 4 + row]
            val = coeffs @ phi
            K_matrix[row, col] = 0.0 if abs(val) < eps else val

    for col in range(10):
        for row in range(10):
            coeffs = A_coeffs[col * 10 + row]
            val = coeffs @ phi
            A_matrix[row, col] = 0.0 if abs(val) < eps else val

    for col in range(4):
        for row in range(10):
            coeffs = B_coeffs[col * 10 + row]
            val = coeffs @ phi
            B_matrix[row, col] = 0.0 if abs(val) < eps else val

    return K_matrix, A_matrix, B_matrix


if __name__ == "__main__":
    ll_grid = np.linspace(0.16000280, 0.38957695, 24)
    lr_grid = np.linspace(0.16000280, 0.38957695, 24)
    theta_lin = 0.0
    dt = 0.002
    q_diag = [500, 5, 2000, 10, 3000, 15, 3000, 15, 16000, 30]
    # q_diag = [1000, 5, 2000, 5, 2000, 5, 500, 1, 500, 1]
    r_diag = [24, 24, 2, 2]

    coeffs = fit_coefficients(ll_grid, lr_grid, q_diag, r_diag, theta_lin, dt)

    output_lines = []

    def format_cpp_decl(name, arr):
        rows, cols = arr.shape
        output_lines.append(f"    inline float {name}[{rows}][{cols}] = {{")
        for i, row in enumerate(arr):
            filtered = [0.0 if abs(v) < 1e-3 else v for v in row]
            row_txt = ", ".join(f"{v:.10g}" for v in filtered)
            if i == rows - 1:
                output_lines.append(f"        {{{row_txt}}}}};")
            else:
                output_lines.append(f"        {{{row_txt}}},")

    format_cpp_decl("K_out", coeffs["K_coeffs"])
    format_cpp_decl("A_out", coeffs["A_coeffs"])
    format_cpp_decl("B_out", coeffs["B_coeffs"])

    full_output = "\n".join(output_lines)

    import subprocess

    try:
        subprocess.run("clip", input=full_output, text=True, check=True, shell=True)
        print("Output copied to clipboard successfully.")
    except Exception as e:
        print(f"Could not copy to clipboard: {e}")
        print(full_output)
