# import itertools
# import math
# from pathlib import Path
# import numpy as np
# import scipy
# import control
# from numpy.linalg import LinAlgError


# def compute_AB(ll, lr, theta_lin=0.0):
#     R_w, R_l = 0.05, 0.1965
#     m_w, m_l, m_b = 0.157, 0.24, 4.96836
#     I_w, I_b, I_z = 0.00019635, 0.06356, 0.114
#     g = 9.8

#     c_lin = math.cos(theta_lin)
#     s_lin = math.sin(theta_lin)

#     l_l, l_r = ll, lr
#     l_wl = 0.00414 * ll + 0.000565
#     l_wr = 0.00414 * lr + 0.000565
#     I_ll = 0.001047 * ll + 0.000134
#     I_lr = 0.001047 * lr + 0.000134

#     inv_Tr = np.matrix([[R_w / 2, R_w / 2, 0, 0, 0],
#                         [-R_w / (2 * R_l), R_w / (2 * R_l), -l_l*c_lin / (2 * R_l), l_r*c_lin / (2 * R_l), 0],
#                         [0, 0, 1, 0, 0],
#                         [0, 0, 0, 1, 0],
#                         [0, 0, 0, 0, 1]])

#     Mat_M = np.matrix([
#         [
#             I_w + I_z*R_w**2/(4*R_l**2) + R_w**2*m_b/4 + R_w**2*m_l + R_w**2*m_w,
#             -I_z*R_w**2/(4*R_l**2) + R_w**2*m_b/4,
#             0.25*I_z*R_w*l_l*c_lin/R_l**2 + 0.25*R_w*l_l*m_b*c_lin + R_w*l_wl*c_lin*m_l,
#             -0.25*I_z*R_w*l_r*c_lin/R_l**2 + 0.25*R_w*l_r*m_b*c_lin,
#             0
#         ],
#         [
#             -I_z*R_w**2/(4*R_l**2) + R_w**2*m_b/4,
#             I_w + I_z*R_w**2/(4*R_l**2) + R_w**2*m_b/4 + R_w**2*m_l + R_w**2*m_w,
#             -0.25*I_z*R_w*l_l*c_lin/R_l**2 + 0.25*R_w*l_l*m_b*c_lin,
#             0.25*I_z*R_w*l_r*c_lin/R_l**2 + 0.25*R_w*l_r*m_b*c_lin + R_w*l_wr*c_lin*m_l,
#             0
#         ],
#         [
#             0.25*I_z*R_w*l_l*c_lin/R_l**2 + 0.25*R_w*l_l*m_b*c_lin + R_w*l_wl*c_lin*m_l,
#             -0.25*I_z*R_w*l_l*c_lin/R_l**2 + 0.25*R_w*l_l*m_b*c_lin,
#             I_ll + 0.25*I_z*l_l**2*c_lin**2/R_l**2 + 0.25*l_l**2*m_b + l_wl**2*m_l,
#             -0.25*I_z*l_l*l_r*c_lin**2/R_l**2 + 0.25*l_l*l_r*m_b,
#             0
#         ],
#         [
#             -0.25*I_z*R_w*l_r*c_lin/R_l**2 + 0.25*R_w*l_r*m_b*c_lin,
#             0.25*I_z*R_w*l_r*c_lin/R_l**2 + 0.25*R_w*l_r*m_b*c_lin + R_w*l_wr*c_lin*m_l,
#             -0.25*I_z*l_l*l_r*c_lin**2/R_l**2 + 0.25*l_l*l_r*m_b,
#             I_lr + 0.25*I_z*l_r**2*c_lin**2/R_l**2 + 0.25*l_r**2*m_b + l_wr**2*m_l,
#             0
#         ],
#         [0, 0, 0, 0, I_b]
#     ])

#     Mat_N = np.matrix([
#         [0, 0, 0, 0, 0],
#         [0, 0, 0, 0, 0],
#         [0, 0, -0.5*g*l_l*m_b*c_lin - g*l_wl*c_lin*m_l, 0, 0],
#         [0, 0, 0, -0.5*g*l_r*m_b*c_lin - g*l_wr*c_lin*m_l, 0],
#         [0, 0, 0, 0, 0]
#     ])

#     Mat_O = np.zeros((5, 5))
#     Mat_O4 = np.zeros((5, 4))
#     Mat_I = np.identity(5)

#     Mat_E = np.matrix([[1, 0, 0, 0],
#                        [0, 1, 0, 0],
#                        [-1, 0, 1, 0],
#                        [0, -1, 0, 1],
#                        [0, 0, -1, -1]])

#     MAT_M = np.matrix(np.block([[Mat_I, Mat_O], [Mat_O, Mat_M]]))
#     MAT_N = np.matrix(np.block([[Mat_O, -Mat_I], [Mat_N, Mat_O]]))
#     MAT_E = np.matrix(np.block([[Mat_O4], [Mat_E]]))
#     MAT_T1 = np.matrix(np.block([[inv_Tr, Mat_O], [Mat_O, inv_Tr]]))
#     MAT_T2 = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
#                         [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
#                         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#                         [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
#                         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
#     MAT_T = MAT_T2 * MAT_T1

#     MAT_A = - MAT_T * MAT_M.I * MAT_N * MAT_T.I
#     MAT_B = MAT_T * MAT_M.I * MAT_E

#     return MAT_A, MAT_B


# def _compute_K(ll, lr, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4,
#                use_discrete=True, dt=0.005, theta_lin=0.0):

#     MAT_A, MAT_B = compute_AB(ll, lr, theta_lin)

#     QQ = np.zeros((10, 10))
#     QQ[0][0] = q1
#     QQ[1][1] = q2
#     QQ[2][2] = q3
#     QQ[3][3] = q4
#     QQ[4][4] = q5
#     QQ[5][5] = q6
#     QQ[6][6] = q7
#     QQ[7][7] = q8
#     QQ[8][8] = q9
#     QQ[9][9] = q10

#     R = np.matrix([[r1, 0, 0, 0],
#                    [0, r2, 0, 0],
#                    [0, 0, r3, 0],
#                    [0, 0, 0, r4]])

#     if use_discrete:
#         def K_discrete_lqr(A, B, Q, R, dt):
#             C = np.zeros((4, 10))
#             D = np.zeros((4, 4))
#             system_LIP = control.ss(A, B, C, D)
#             system_dLIP = control.c2d(system_LIP, dt)
#             MAT_dA = np.matrix(system_dLIP.A)
#             MAT_dB = np.matrix(system_dLIP.B)
#             P = scipy.linalg.solve_discrete_are(MAT_dA, MAT_dB, Q, R)
#             K = (R + MAT_dB.T * P * MAT_dB).I * MAT_dB.T * P * MAT_dA
#             return K

#         K_gain = K_discrete_lqr(MAT_A, MAT_B, QQ, R, dt)
#     else:
#         P = scipy.linalg.solve_continuous_are(MAT_A, MAT_B, QQ, R)
#         K_gain = R.I * MAT_B.T * P

#     return K_gain


# def _poly_features(ll, lr):
#     return np.array([1.0, ll, lr, ll * ll, ll * lr, lr * lr], dtype=float)


# def _fit_matrix_coeffs(samples, rows, cols):
#     X = np.vstack([_poly_features(ll, lr) for ll, lr, _ in samples])
#     coeffs = np.zeros((rows * cols, 6))
#     for c in range(cols):
#         for r in range(rows):
#             y = np.array([mat[r, c] for _, _, mat in samples], dtype=float)
#             beta, *_ = np.linalg.lstsq(X, y, rcond=None)
#             coeffs[c * rows + r, :] = beta
#     return coeffs


# def fit_coefficients(ll_values, lr_values, q_diag, r_diag, theta_lin=0.0, dt=0.001):
#     from tqdm import tqdm

#     samples_K = []
#     samples_A = []
#     samples_B = []

#     q1, q2, q3, q4, q5, q6, q7, q8, q9, q10 = q_diag
#     r1, r2, r3, r4 = r_diag

#     grid_size = len(ll_values) * len(lr_values)
#     print(f"Total iterations: {grid_size}. This might take a few seconds...")
#     for ll, lr in tqdm(itertools.product(ll_values, lr_values), total=grid_size, desc="Solving LQR"):
#         A_mat, B_mat = compute_AB(ll, lr, theta_lin)
#         try:
#             K_mat = _compute_K(ll, lr, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4, theta_lin=theta_lin, dt=dt)
#         except LinAlgError:
#             continue
#         samples_A.append((ll, lr, np.asarray(A_mat, dtype=float)))
#         samples_B.append((ll, lr, np.asarray(B_mat, dtype=float)))
#         samples_K.append((ll, lr, np.asarray(K_mat, dtype=float)))

#     if not samples_K:
#         raise RuntimeError("No stable samples available; adjust Q/R or sampling grid.")

#     K_coeffs = _fit_matrix_coeffs(samples_K, 4, 10)
#     A_coeffs = _fit_matrix_coeffs(samples_A, 10, 10)
#     B_coeffs = _fit_matrix_coeffs(samples_B, 10, 4)

#     return {
#         "K_coeffs": K_coeffs,
#         "A_coeffs": A_coeffs,
#         "B_coeffs": B_coeffs,
#     }


# def _format_cpp_array(arr, eps=1e-3):
#     lines = []
#     for row in arr:
#         filtered = [0.0 if abs(v) < eps else v for v in row]
#         row_txt = ", ".join(f"{v:.10g}" for v in filtered)
#         lines.append(f"    {{{row_txt}}}")
#     return "{\n" + ",\n".join(lines) + "\n}"


# def export_cpp_library(coeffs, header_path="lqr_coeffs.h", source_path="lqr_coeffs.cpp", namespace="lqr_fit"):

#     header_path = Path(header_path)
#     source_path = Path(source_path)

#     K_coeffs = np.asarray(coeffs["K_coeffs"], dtype=float)
#     A_coeffs = np.asarray(coeffs["A_coeffs"], dtype=float)
#     B_coeffs = np.asarray(coeffs["B_coeffs"], dtype=float)

#     header = f"""#pragma once
# #include <cstddef>

# namespace {namespace} {{

# extern const double K_out[40][6];
# extern const double A_out[100][6];
# extern const double B_out[40][6];

# void lqr_KAB(double LL, double LR,
#              double K_matrix[4][10],
#              double A_matrix[10][10],
#              double B_matrix[10][4]);

# }}

# """

#     source = f"""#include <array>
# #include "{header_path.name}"

# namespace {namespace} {{

# const double K_out[40][6] = {_format_cpp_array(K_coeffs)};
# const double A_out[100][6] = {_format_cpp_array(A_coeffs)};
# const double B_out[40][6] = {_format_cpp_array(B_coeffs)};

# void lqr_KAB(double LL, double LR,
#              double K_matrix[4][10],
#              double A_matrix[10][10],
#              double B_matrix[10][4]) {{
#     double phi[6] = {{1.0, LL, LR, LL * LL, LL * LR, LR * LR}};
#     const double eps = 1e-3;

#     for (std::size_t col = 0; col < 10; ++col) {{
#         for (std::size_t row = 0; row < 4; ++row) {{
#             const double* coeffs = K_out[col * 4 + row];
#             double val = 0.0;
#             for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
#             K_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
#         }}
#     }}

#     for (std::size_t col = 0; col < 10; ++col) {{
#         for (std::size_t row = 0; row < 10; ++row) {{
#             const double* coeffs = A_out[col * 10 + row];
#             double val = 0.0;
#             for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
#             A_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
#         }}
#     }}

#     for (std::size_t col = 0; col < 4; ++col) {{
#         for (std::size_t row = 0; row < 10; ++row) {{
#             const double* coeffs = B_out[col * 10 + row];
#             double val = 0.0;
#             for (std::size_t i = 0; i < 6; ++i) val += coeffs[i] * phi[i];
#             B_matrix[row][col] = (val < eps && val > -eps) ? 0.0 : val;
#         }}
#     }}
# }}

# }}

# """

#     header_path.write_text(header, encoding="utf-8")
#     source_path.write_text(source, encoding="utf-8")
#     return header_path, source_path


# def lqr_KAB(LL, LR, K_coeffs, A_coeffs, B_coeffs):
#     K_matrix = np.zeros((4, 10))
#     A_matrix = np.zeros((10, 10))
#     B_matrix = np.zeros((10, 4))
#     phi = _poly_features(LL, LR)
#     eps = 1e-3

#     for col in range(10):
#         for row in range(4):
#             coeffs = K_coeffs[col * 4 + row]
#             val = coeffs @ phi
#             K_matrix[row, col] = 0.0 if abs(val) < eps else val

#     for col in range(10):
#         for row in range(10):
#             coeffs = A_coeffs[col * 10 + row]
#             val = coeffs @ phi
#             A_matrix[row, col] = 0.0 if abs(val) < eps else val

#     for col in range(4):
#         for row in range(10):
#             coeffs = B_coeffs[col * 10 + row]
#             val = coeffs @ phi
#             B_matrix[row, col] = 0.0 if abs(val) < eps else val

#     return K_matrix, A_matrix, B_matrix


# if __name__ == "__main__":
#     ll_grid = np.linspace(0.15, 0.3, 20)
#     lr_grid = np.linspace(0.15, 0.3, 20)
#     theta_lin = 0.0
#     dt = 0.005
#     q_diag = [5, 120, 300, 12, 36000, 16, 36000, 16, 14000, 12]
#     r_diag = [1, 1, 0.25, 0.25]

#     coeffs = fit_coefficients(ll_grid, lr_grid, q_diag, r_diag, theta_lin, dt)

#     output_lines = []

#     def format_cpp_decl(name, arr):
#         rows, cols = arr.shape
#         output_lines.append(f"    inline float {name}[{rows}][{cols}] = {{")
#         for i, row in enumerate(arr):
#             filtered = [0.0 if abs(v) < 1e-3 else v for v in row]
#             row_txt = ", ".join(f"{v:.10g}" for v in filtered)
#             if i == rows - 1:
#                 output_lines.append(f"        {{{row_txt}}}}};")
#             else:
#                 output_lines.append(f"        {{{row_txt}}},")

#     format_cpp_decl("K_out", coeffs["K_coeffs"])
#     format_cpp_decl("A_out", coeffs["A_coeffs"])
#     format_cpp_decl("B_out", coeffs["B_coeffs"])

#     full_output = "\n".join(output_lines)

#     import subprocess
#     try:
#         subprocess.run('clip', input=full_output, text=True, check=True, shell=True)
#         print("Output copied to clipboard successfully.")
#     except Exception as e:
#         print(f"Could not copy to clipboard: {e}")
#         print(full_output)


"""用当前控制器的 lagrange.K() 拟合 K，并导出单片机 C 代码。

把本文件放在当前控制器的 lagrange.py 旁边，用同一个 Python 环境运行：
    python fit_lqr_coeffs.py
也可明确指定模型：
    python fit_lqr_coeffs.py --model path/to/lagrange.py --output lqr_export

默认从模型 LEG_PARAMETERS 自动读取完整范围：0.16000280～0.38957695 m。
在倒数第二个查表点处分段，左右腿分别选择区间，使用五次二元多项式。
Q/R 与当前 10 状态控制器一致。这里只拟合 K，不另外重建机器人 A/B，
不改变模型使用的连续/离散 LQR，也不把电机换号、PID、VMC 算进 K。

输出：lqr_gain.h、lqr_gain.c、lqr_gain.py、lqr_fit.json、README.md。
修改模型、Q/R 或使用腿长范围后，需要重新生成。
拟合存在近似误差；数值校验通过不代表实机闭环已经验证。
"""

import argparse
import ast
import hashlib
import importlib.util
import inspect
import itertools
import json
import sys
from pathlib import Path

import numpy as np


# 与当前控制器 lagrange.K(...) 的 14 个权重参数完全一致。
Q_DIAG = [1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10]
R_DIAG = [1, 1, 1, 1]

# 默认拟合模型查表的全部腿长，不能把控制器 final_L0=0.22 当成模型上限。
SAMPLES_PER_AXIS = 31
POLY_DEGREE = 5

# 校验量：max|误差_ij| / max(1, max|精确 K_ij|)。不是闭环稳定性指标。
MAX_SCALED_ERROR = 0.002

STATE_ORDER = [
    "s",
    "dot_s_b",
    "fai",
    "dot_fai",
    "theta_ll",
    "dot_theta_ll",
    "theta_lr",
    "dot_theta_lr",
    "theta_b",
    "dot_theta_b",
]
OUTPUT_ORDER = ["T_l", "T_r", "T_pl", "T_pr"]


def load_model(model_path):
    """只读取明确指定的模型，不回退到旧机器人参数。"""
    model_path = Path(model_path).resolve()
    if not model_path.is_file():
        raise FileNotFoundError(
            f"未找到 {model_path}\n请把脚本放在正在使用的 lagrange.py 旁边，"
            "或用 --model 指定该文件。"
        )
    source = model_path.read_bytes()
    spec = importlib.util.spec_from_file_location("_lqr_source_model", model_path)
    model = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = model
    sys.path.insert(0, str(model_path.parent))
    # 直接执行这次读取的源文件，避免缓存的 pyc 与记录的源文件哈希不一致。
    exec(compile(source, str(model_path), "exec"), model.__dict__)
    if not callable(getattr(model, "K", None)):
        raise ValueError("模型文件必须提供与控制器相同的 K(ll, lr, *Q, *R) 函数")
    return model, hashlib.sha256(source).hexdigest()


def _compute_K(model, ll, lr, q_diag, r_diag):
    try:
        K = np.asarray(model.K(float(ll), float(lr), *q_diag, *r_diag), dtype=float)
    except Exception as exc:
        raise RuntimeError(
            f"lagrange.K 在 LL={ll:.8f}, LR={lr:.8f} m 求解失败"
        ) from exc
    if K.shape != (4, 10) or not np.all(np.isfinite(K)):
        raise ValueError(f"LL={ll:.8f}, LR={lr:.8f} 的 K 必须是有限的 4×10 矩阵")
    return K


def _powers(degree):
    # 三次时的顺序：1, x, y, x², xy, y², x³, x²y, xy², y³。
    return [(n - j, j) for n in range(degree + 1) for j in range(n + 1)]


def _poly_features(ll, lr, config, dtype=np.float64):
    # 腿长先归一化，避免直接拟合 m、m²、m³ 导致各列数量级差别很大。
    centers = np.asarray(config["length_center"], dtype=dtype)
    inv_scales = np.asarray(config["length_inv_scale"], dtype=dtype)
    x = (dtype(ll) - centers[0]) * inv_scales[0]
    y = (dtype(lr) - centers[1]) * inv_scales[1]
    xp, yp = [dtype(1)], [dtype(1)]
    for _ in range(config["degree"]):
        xp.append(xp[-1] * x)
        yp.append(yp[-1] * y)
    return np.array([xp[i] * yp[j] for i, j in config["powers"]], dtype=dtype)


def _patch_config(config, il, ir):
    centers = config["centers"]
    scales = config["inv_scales"]
    return dict(
        config,
        length_center=[centers[il], centers[ir]],
        length_inv_scale=[scales[il], scales[ir]],
    )


def _patch_K(ll, lr, coeffs, patch):
    features = _poly_features(ll, lr, patch, np.float32)
    K = np.zeros((4, 10), dtype=np.float32)
    for k in range(len(features)):
        K += coeffs[:, :, k] * features[k]
    return K


def fitted_K(ll, lr, coeffs, config):
    """模拟 C float 运算；先按左右腿长分别选区间，再算 K[4][10]。"""
    breaks = np.asarray(config["breaks"], dtype=np.float32)
    ll, lr = np.float32(ll), np.float32(lr)
    if not (
        np.isfinite(ll)
        and np.isfinite(lr)
        and breaks[0] <= ll <= breaks[-1]
        and breaks[0] <= lr <= breaks[-1]
    ):
        raise ValueError("腿长非法或超出拟合范围")
    il = min(int(np.searchsorted(breaks[1:], ll, side="right")), len(breaks) - 2)
    ir = min(int(np.searchsorted(breaks[1:], lr, side="right")), len(breaks) - 2)
    return _patch_K(ll, lr, coeffs[il, ir], _patch_config(config, il, ir))


def reference_dynamics(model):
    """为检查 A-BK，复用源 K 函数中 CARE 之前的 A/B 计算语句。

    不改写 lagrange.py，也不另抄一套物理参数。仅支持本次提供的函数结构。
    """
    function = ast.parse(inspect.getsource(model.K)).body[0]
    prefix = []
    for statement in function.body:
        if (
            isinstance(statement, ast.Assign)
            and isinstance(statement.value, ast.Call)
            and isinstance(statement.value.func, ast.Attribute)
            and statement.value.func.attr == "solve_continuous_are"
        ):
            break
        prefix.append(statement)
    else:
        raise ValueError("模型结构已变化：找不到连续 CARE，需同步修改 A/B 校验接口")
    function.name = "_reference_AB"
    function.body = prefix + [
        ast.Return(
            value=ast.Tuple(
                elts=[
                    ast.Name(id="MAT_A", ctx=ast.Load()),
                    ast.Name(id="MAT_B", ctx=ast.Load()),
                ],
                ctx=ast.Load(),
            )
        )
    ]
    tree = ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[]))
    namespace = dict(model.__dict__)
    exec(compile(tree, str(model.__file__), "exec"), namespace)
    return namespace[function.name]


def fit_coefficients(model, config):
    breaks = np.asarray(config["breaks"])
    table_lengths = np.asarray(model.LEG_PARAMETERS)[:, 0]
    segment_count = len(breaks) - 1
    grids = []
    for lo, hi in zip(breaks[:-1], breaks[1:]):
        knots = table_lengths[(lo <= table_lengths) & (table_lengths <= hi)]
        grids.append(
            np.unique(np.r_[np.linspace(lo, hi, config["samples_per_axis"]), knots])
        )
    coeffs = np.empty(
        (segment_count, segment_count, 4, 10, len(config["powers"])), dtype=np.float32
    )
    points = []
    for il, ir in itertools.product(range(segment_count), repeat=2):
        sample_points = list(itertools.product(grids[il], grids[ir]))
        print(
            f"拟合区间 ({il + 1}, {ir + 1})：{len(sample_points)} 组腿长……", flush=True
        )
        patch = _patch_config(config, il, ir)
        samples = np.array(
            [
                _compute_K(model, ll, lr, config["q_diag"], config["r_diag"])
                for ll, lr in sample_points
            ]
        )
        features = np.array([_poly_features(ll, lr, patch) for ll, lr in sample_points])
        beta, _, rank, _ = np.linalg.lstsq(
            features, samples.reshape(-1, 40), rcond=None
        )
        if rank != len(config["powers"]):
            raise ValueError("拟合矩阵不满秩，请增加采样数或降低多项式阶数")
        coeffs[il, ir] = beta.T.reshape(4, 10, -1)
        points.extend(sample_points)
    if not np.all(np.isfinite(coeffs)):
        raise ValueError("拟合系数无法表示为有限的 float32")

    # 用训练网格中点、所有原始查表节点、分段边界和随机点验证。
    lengths = np.unique(np.concatenate(grids))
    midpoints = (lengths[:-1] + lengths[1:]) / 2
    check_lengths = np.unique(np.r_[midpoints, table_lengths, breaks])
    check_lengths = check_lengths[
        (breaks[0] <= check_lengths) & (check_lengths <= breaks[-1])
    ]
    checks = np.array(list(itertools.product(check_lengths, repeat=2)))
    random_points = np.random.default_rng(0).uniform(breaks[0], breaks[-1], (256, 2))
    checks = np.vstack((checks, random_points))
    print(f"用 {len(checks)} 组验证点检查 float32 拟合结果……", flush=True)
    exact = np.array(
        [
            _compute_K(model, ll, lr, config["q_diag"], config["r_diag"])
            for ll, lr in checks
        ]
    )
    fitted = np.array([fitted_K(ll, lr, coeffs, config) for ll, lr in checks])
    error = fitted.astype(float) - exact
    abs_error = np.abs(error)
    peak = np.maximum(1.0, np.max(np.abs(exact), axis=0))
    per_gain_max = np.max(abs_error, axis=0)
    worst = np.unravel_index(np.argmax(abs_error), abs_error.shape)
    sample, row, col = (int(n) for n in worst)
    # 分段边界两侧各自的极限值，检查切换带来的 K 跳变量。
    seam_jump = 0.0
    for seam in range(1, segment_count):
        fixed = breaks[seam]
        for other in range(segment_count):
            for value in np.linspace(breaks[other], breaks[other + 1], 31):
                for ll, lr, a, b in [
                    (fixed, value, (seam - 1, other), (seam, other)),
                    (value, fixed, (other, seam - 1), (other, seam)),
                ]:
                    ka = _patch_K(ll, lr, coeffs[a], _patch_config(config, *a))
                    kb = _patch_K(ll, lr, coeffs[b], _patch_config(config, *b))
                    seam_jump = max(seam_jump, float(np.max(np.abs(ka - kb))))

    print("检查每个验证点的冻结腿长连续模型 A-BK……", flush=True)
    get_ab = reference_dynamics(model)
    exact_alpha, fitted_alpha = [], []
    for p, k_exact, k_fit in zip(checks, exact, fitted):
        A, B = get_ab(float(p[0]), float(p[1]), *config["q_diag"], *config["r_diag"])
        A, B = np.asarray(A), np.asarray(B)
        exact_alpha.append(float(np.max(np.linalg.eigvals(A - B @ k_exact).real)))
        fitted_alpha.append(float(np.max(np.linalg.eigvals(A - B @ k_fit).real)))
    validation = {
        "training_points": len(points),
        "validation_points": len(checks),
        "coefficient_dtype": "float32",
        "evaluation_dtype": "float32",
        "max_abs_error": float(np.max(abs_error)),
        "rms_error": float(np.sqrt(np.mean(error**2))),
        "max_scaled_error": float(np.max(per_gain_max / peak)),
        "scaled_error_definition": "max_abs_error_ij / max(1, peak_abs_exact_K_ij)",
        "per_gain_max_abs_error": per_gain_max.tolist(),
        "per_gain_max_scaled_error": (per_gain_max / peak).tolist(),
        "worst_abs_error_point": {
            "LL": float(checks[sample, 0]),
            "LR": float(checks[sample, 1]),
            "output": OUTPUT_ORDER[row],
            "state": STATE_ORDER[col],
            "row": row,
            "column": col,
            "exact": float(exact[sample, row, col]),
            "fitted": float(fitted[sample, row, col]),
        },
        "max_gain_jump_at_segment_boundary": seam_jump,
        "closed_loop_stability_checked": True,
        "stability_scope": "continuous frozen-leg-length A-BK from supplied lagrange.K only",
        "exact_max_eigenvalue_real": max(exact_alpha),
        "fitted_max_eigenvalue_real": max(fitted_alpha),
        "unstable_validation_points": sum(v >= 0 for v in fitted_alpha),
    }
    print(f"最大绝对误差：{validation['max_abs_error']:.8g}")
    print(f"最大混合归一误差：{validation['max_scaled_error']:.4%}")
    print(f"分段边界最大增益跳变量：{seam_jump:.8g}")
    print(f"拟合闭环最大特征值实部：{max(fitted_alpha):.8g}")
    if validation["max_scaled_error"] > config["max_scaled_error"]:
        raise ValueError(
            "拟合误差超过设定值，本次不导出；请缩小腿长范围或提高 --degree"
        )
    if validation["unstable_validation_points"]:
        raise ValueError("存在 A-BK 不稳定的验证点，本次不导出")
    return coeffs, validation


def _c_float(value):
    # 9 位有效数字可保存 float32 值，不随意把小系数置零。
    text = format(float(np.float32(value)), ".9g")
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def _c_array(values, indent=0):
    values = np.asarray(values)
    if values.ndim == 1:
        return "{" + ", ".join(_c_float(v) for v in values) + "}"
    pad = "    " * (indent + 1)
    return (
        "{\n"
        + ",\n".join(pad + _c_array(v, indent + 1) for v in values)
        + "\n"
        + "    " * indent
        + "}"
    )


def export_c_library(coeffs, config, validation, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    terms = len(config["powers"])
    segments = len(config["breaks"]) - 1
    stamp = (
        f"/* Generated from lagrange.K; model SHA256: {config['model_sha256']}\n"
        f" * Q: {config['q_diag']}\n * R: {config['r_diag']}\n"
        " * K is approximate; changing model or weights requires regeneration. */\n"
    )
    header = (
        stamp
        + f"""
#ifndef LQR_GAIN_H
#define LQR_GAIN_H

#ifdef __cplusplus
extern "C" {{
#endif

#define LQR_STATE_COUNT 10
#define LQR_OUTPUT_COUNT 4
#define LQR_SEGMENT_COUNT {segments}
#define LQR_LENGTH_MIN {_c_float(config["length_min"])}
#define LQR_LENGTH_MAX {_c_float(config["length_max"])}

/* State order (SI units):
 * s, dot_s_b, fai, dot_fai, theta_ll, dot_theta_ll,
 * theta_lr, dot_theta_lr, theta_b, dot_theta_b.
 * All angles: rad; angular velocities: rad/s.
 * Output order: T_l, T_r, T_pl, T_pr (N*m).
 * T_pl/T_pr are virtual leg swing torques, not direct hip motor commands.
 */

/* LL/LR: target leg lengths in metres, as in the current controller.
 * Return 0 on success; -1 for non-finite/out-of-range lengths.
 * On error K is unchanged; the caller must handle the error before using K.
 * No extrapolation. Output layout: K[output][state]. */
int lqr_get_k(float LL, float LR, float K[4][10]);

/* U = K * (expect_state - real_state).
 * No extra minus sign and no actuator saturation applied here. */
void lqr_compute_u(const float K[4][10], const float expect_state[10],
                   const float real_state[10], float U[4]);

#ifdef __cplusplus
}}
#endif
#endif
"""
    )
    coeff_text = _c_array(coeffs)
    features = ", ".join(f"xp[{i}] * yp[{j}]" for i, j in config["powers"])
    source = (
        stamp
        + f"""
#include "lqr_gain.h"
#include <math.h>

/* First two indices select left/right length segments independently. */
static const float K_coeffs[{segments}][{segments}][4][10][{terms}] = {coeff_text};
static const float length_breaks[{segments + 1}] = {_c_array(config["breaks"])};
static const float centers[{segments}] = {_c_array(config["centers"])};
static const float inv_scales[{segments}] = {_c_array(config["inv_scales"])};

int lqr_get_k(float LL, float LR, float K[4][10]) {{
    if (!isfinite(LL) || !isfinite(LR) ||
        LL < LQR_LENGTH_MIN || LL > LQR_LENGTH_MAX ||
        LR < LQR_LENGTH_MIN || LR > LQR_LENGTH_MAX)
        return -1;

    int il = 0, ir = 0;
    while (il < LQR_SEGMENT_COUNT - 1 && LL >= length_breaks[il + 1]) ++il;
    while (ir < LQR_SEGMENT_COUNT - 1 && LR >= length_breaks[ir + 1]) ++ir;
    float x = (LL - centers[il]) * inv_scales[il];
    float y = (LR - centers[ir]) * inv_scales[ir];
    float xp[{config["degree"] + 1}] = {{1.0f}};
    float yp[{config["degree"] + 1}] = {{1.0f}};
    for (int i = 1; i <= {config["degree"]}; ++i) {{
        xp[i] = xp[i - 1] * x;
        yp[i] = yp[i - 1] * y;
    }}
    float features[{terms}] = {{{features}}};
    for (int row = 0; row < 4; ++row) {{
        for (int col = 0; col < 10; ++col) {{
            float value = 0.0f;
            for (int k = 0; k < {terms}; ++k)
                value += K_coeffs[il][ir][row][col][k] * features[k];
            K[row][col] = value;
        }}
    }}
    return 0;
}}

void lqr_compute_u(const float K[4][10], const float expect_state[10],
                   const float real_state[10], float U[4]) {{
    for (int row = 0; row < 4; ++row) {{
        U[row] = 0.0f;
        for (int col = 0; col < 10; ++col)
            U[row] += K[row][col] * (expect_state[col] - real_state[col]);
    }}
}}
"""
    )
    python = '''"""拟合版 lagrange.K：将本文件和 lqr_fit.json 放到控制器目录。"""
import json
from pathlib import Path
import numpy as np

_data = json.loads(Path(__file__).with_name("lqr_fit.json").read_text(encoding="utf-8"))
_config = _data["config"]
_coeffs = np.array(_data["K_coeffs"], dtype=np.float32)
_breaks = np.array(_config["breaks"], dtype=np.float32)
_centers = np.array(_config["centers"], dtype=np.float32)
_inv_scales = np.array(_config["inv_scales"], dtype=np.float32)


def K(ll, lr, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4):
    weights = [q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4]
    if weights != _config["q_diag"] + _config["r_diag"]:
        raise ValueError("Q/R 与生成系数时不同，请重新拟合")
    ll, lr = np.float32(ll), np.float32(lr)
    lo, hi = np.float32(_config["length_min"]), np.float32(_config["length_max"])
    if not (np.isfinite(ll) and np.isfinite(lr) and lo <= ll <= hi and lo <= lr <= hi):
        raise ValueError(f"目标腿长必须位于 [{lo}, {hi}] m")
    il = min(int(np.searchsorted(_breaks[1:], ll, side="right")), len(_breaks) - 2)
    ir = min(int(np.searchsorted(_breaks[1:], lr, side="right")), len(_breaks) - 2)
    x = (ll - _centers[il]) * _inv_scales[il]
    y = (lr - _centers[ir]) * _inv_scales[ir]
    xp, yp = [np.float32(1)], [np.float32(1)]
    for _ in range(_config["degree"]):
        xp.append(xp[-1] * x)
        yp.append(yp[-1] * y)
    result = np.zeros((4, 10), dtype=np.float32)
    for k, (i, j) in enumerate(_config["powers"]):
        result += _coeffs[il, ir, :, :, k] * (xp[i] * yp[j])
    return np.matrix(result)
'''
    readme = f"""# 当前控制器的 K 拟合结果

模型文件：`{Path(config["model_path"]).name}`。SHA256：`{config["model_sha256"]}`。
控制器和运动学文件的来源及哈希也记录在 lqr_fit.json 中。
Q = `{config["q_diag"]}`，R = `{config["r_diag"]}`。
左右目标腿长范围均为 {config["length_min"]:.8f}～{config["length_max"]:.8f} m。
分段点：`{config["breaks"]}`。左右腿分别选段，共 {segments * segments} 种区间组合。
每段是归一化二元 {config["degree"]} 次多项式，每个增益 {terms} 项，40 个增益；
系数占用 {coeffs.nbytes} 字节（float32）。不额外调整连续/离散模型及采样周期。
拟合的是直接求解的 `lagrange.K(LL, LR, *Q, *R)`，并非两个端点 K 的直线插值。

## 使用顺序

1. 在 Webots 中，将 `lqr_gain.py` 和 `lqr_fit.json` 放到控制器旁边，
   把 `import lagrange` 改为 `import lqr_gain as lagrange`。
   你本次上传的控制器仍然是 `K=(1-leg_ratio)*K_start+leg_ratio*K_final`。
   要使用每拍腿长对应的拟合 K，还需要把这一行替换为：

```python
K = lagrange.K(target_L0_l, target_L0_r,
               {", ".join(str(v) for v in config["q_diag"] + config["r_diag"])})
U = K * (expect_state - real_state)
```

   循环外的 K_start、K_final 计算可以删除；目标腿长设置、腿长 PD、VMC、
   传感器换算和限幅保持原写法。原来的 final_L0=0.22 不会因换拟合器自动增大。
2. 验证相同的起立、变腿长、转向流程后，将 `lqr_gain.h` 和 `lqr_gain.c`
   加入单片机工程（C99），不需要 Python、SciPy 或动态内存。
3. 单片机每个控制周期先更新目标腿长，再调用以下函数。

```c
float K[4][10];
float U[4];
/* expect_state 和 real_state 必须按下述顺序构造。 */
int status = lqr_get_k(target_L0_l, target_L0_r, K);
if (status == 0) {{
    lqr_compute_u(K, expect_state, real_state, U);
    /* U[0]=T_l, U[1]=T_r, U[2]=T_pl, U[3]=T_pr。 */
}}
/* status == -1 表示长度非法/越界；本拍不能使用未更新的 K。 */
```

状态顺序：`{", ".join(STATE_ORDER)}`。
前两个量分别为 m、m/s，其余为 rad、rad/s 交替排列。
输出顺序：`{", ".join(OUTPUT_ORDER)}`，单位 N*m。
控制律为 `U = K * (expect_state - real_state)`，没有第二次取负号。
这是行优先的 `K[4][10]`；不能套用旧脚本 `col*4+row` 的系数读取方式。

当前 Webots 控制器轮力矩直接使用 U[0]、U[1]，轮编码器也直接读取。
U[2]、U[3] 必须继续分别和 F_bl、F_br 经 JRM 映射成四个髋电机力矩。
你本次上传控制器启用的是腿长 PD（kp=400、kd=40，直接对腿长速度加阻尼），
没有启用前面对话中的 PID(400,2,8000)。该 PD、22 N 前馈、40 N 腿力限幅、
10 N*m 髋限幅、0.5 N*m 轮限幅
属于控制器，不包括在 K 中。实机电机安装方向和编码器零点应转换到相同坐标。
保持角度零点、速度计算方法和控制周期，不能只移植 K 数值。

## 本次文件中已有的差别

- 最后一个附件与第一个附件是字节完全相同的 lagrange.py；本包未修改这两个模型。
- 模型轮半径是 0.060000003 m，控制器里是 0.05995 m，约差 0.0833%。
- 模型 inv_Tr 第一行只含左右轮角，定义的是轮轴平均位移；控制器使用含腿摆项的
  dot_s_b，低速时积分为 s_local，高速时清零。两者的位移状态定义并不严格相同。
  本次只拟合原 K，没有暗中改变这套已经能运行的模型/控制器；移植前应统一这些定义。
- 已核对 leg.py 的 getPhi、spd、Mat_JRM 与控制器的参数顺序；五连杆的杆长仍为
  0.21、0.25、0.25、0.21、0 m，虚拟腿角仍是 pi/2-phi0+theta_b。
  leg.py 用于计算状态和 VMC，不把 JRM 或四个髋电机极性额外乘入 K。
- 本次提供的 leg.spd 内有 t106=1/t103、t107=1/(t103*t103)，精确竖直时
  t103 可能为零。25 个腿长 × 3 个姿态的独立检查中，9 个精确竖直点触发除零。
  这不影响离线 lagrange.K 拟合，但速度解算移植时需要修正这个可去奇点；
  本包保留原 leg.py，不把这个问题误判为 K 拟合失败。

## 数值验证

训练 {validation["training_points"]} 点，验证 {validation["validation_points"]} 点
（网格中点、原始查表节点、边界、随机点），验证包含 float32 系数及计算误差。
最大绝对增益误差：{validation["max_abs_error"]:.8g}；
最大混合归一误差：{validation["max_scaled_error"]:.6%}。
混合归一量是每个增益的最大绝对误差除以 `max(1, 该增益的精确值绝对峰值)`，
不是每个瞬时增益的相对误差，也不是力矩或闭环误差。
分段边界两侧最大增益跳变量：{validation["max_gain_jump_at_segment_boundary"]:.8g}。
精确 K 的 A-BK 最大特征值实部：{validation["exact_max_eigenvalue_real"]:.8g}；
拟合 K 的 A-BK 最大特征值实部：{validation["fitted_max_eigenvalue_real"]:.8g}。
检查通过的含义是：这些验证点上，原 lagrange 模型在固定腿长时的连续线性闭环稳定。
它不证明连续整个区域、快速变腿长、限幅后的非线性控制器或实机都稳定，
也不包含上面提到的控制器位移状态定义差别。
模型如果依赖其他参数文件，修改这些文件后也必须重新生成。
模型求解或误差校验失败时不会导出本次结果；已有输出不代表本次配置已更新。
"""
    report = {"config": config, "validation": validation, "K_coeffs": coeffs.tolist()}
    for name, content in [
        ("lqr_gain.h", header),
        ("lqr_gain.c", source),
        ("lqr_gain.py", python),
        ("lqr_fit.json", json.dumps(report, ensure_ascii=False, indent=2)),
        ("README.md", readme),
    ]:
        (output_dir / name).write_text(content, encoding="utf-8")
    print(f"已导出到：{output_dir.resolve()}")


def read_controller(controller_path):
    """用 AST 读取实际启用的 K 调用和状态顺序，不运行 Webots 控制器。"""
    path = Path(controller_path)
    tree = ast.parse(path.read_text(encoding="utf-8-sig"))
    weights = []
    state_order = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "lagrange"
            and node.func.attr == "K"
        ):
            if len(node.args) != 16:
                raise ValueError(
                    "控制器的 K 参数数量已变化，请核对 2 个腿长和 14 个权重"
                )
            weights.append([ast.literal_eval(arg) for arg in node.args[2:]])
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "real_state" for t in node.targets
        ):
            state_order = [row.elts[0].id for row in node.value.args[0].elts]
    if not weights or any(w != weights[0] for w in weights):
        raise ValueError("控制器没有唯一一致的 Q/R；不同权重需要分别生成系数")
    if state_order != STATE_ORDER:
        raise ValueError(f"控制器状态顺序与本脚本不同：{state_order}")
    return (
        weights[0][:10],
        weights[0][10:],
        {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "state_order": state_order,
        },
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, default=Path(__file__).with_name("lagrange.py")
    )
    parser.add_argument(
        "--output", type=Path, default=Path(__file__).with_name("lqr_export")
    )
    parser.add_argument(
        "--controller", type=Path, default=Path(__file__).with_name("my_controller.py")
    )
    parser.add_argument("--leg", type=Path, default=Path(__file__).with_name("leg.py"))
    parser.add_argument("--min-length", type=float, default=None)
    parser.add_argument("--max-length", type=float, default=None)
    parser.add_argument("--samples", type=int, default=SAMPLES_PER_AXIS)
    parser.add_argument("--degree", type=int, default=POLY_DEGREE)
    args = parser.parse_args()

    model, model_sha256 = load_model(args.model)
    table_lengths = np.asarray(model.LEG_PARAMETERS, dtype=float)[:, 0]
    if not (
        len(table_lengths) >= 3
        and np.all(np.isfinite(table_lengths))
        and np.all(np.diff(table_lengths) > 0)
    ):
        parser.error("模型 LEG_PARAMETERS 腿长必须是严格递增的有限数值")
    if args.min_length is None:
        args.min_length = float(table_lengths[0])
    if args.max_length is None:
        args.max_length = float(table_lengths[-1])
    if not (
        np.isfinite(args.min_length)
        and np.isfinite(args.max_length)
        and 0 < args.min_length < args.max_length
        and table_lengths[0] <= args.min_length
        and args.max_length <= table_lengths[-1]
    ):
        parser.error("要求 min-length < max-length，且在模型 LEG_PARAMETERS 范围内")
    if not (1 <= args.degree <= 7 and args.samples >= args.degree + 2):
        parser.error("degree 支持 1～7；samples 至少为 degree + 2")
    q_diag, r_diag, controller = Q_DIAG, R_DIAG, None
    if args.controller.is_file():
        q_diag, r_diag, controller = read_controller(args.controller)
    else:
        print("未找到控制器文件，使用顶部已核对的 Q_DIAG、R_DIAG。", flush=True)
    if (
        len(q_diag) != 10
        or len(r_diag) != 4
        or not np.all(np.isfinite(q_diag + r_diag))
        or min(q_diag) < 0
        or min(r_diag) <= 0
    ):
        parser.error("Q 必须有 10 个非负权重，R 必须有 4 个正权重，全部必须有限")
    breaks = [args.min_length]
    # 最后一段数据斜率有明显变化，因此在倒数第二个原始节点单独分段。
    if args.min_length < table_lengths[-2] < args.max_length:
        breaks.append(float(table_lengths[-2]))
    breaks.append(args.max_length)
    centers = [(lo + hi) / 2 for lo, hi in zip(breaks[:-1], breaks[1:])]
    inv_scales = [2 / (hi - lo) for lo, hi in zip(breaks[:-1], breaks[1:])]
    config = {
        "model_path": str(args.model.resolve()),
        "model_sha256": model_sha256,
        "q_diag": q_diag,
        "r_diag": r_diag,
        "controller": controller,
        "kinematics": (
            {
                "path": str(args.leg.resolve()),
                "sha256": hashlib.sha256(args.leg.read_bytes()).hexdigest(),
            }
            if args.leg.is_file()
            else None
        ),
        "length_min": args.min_length,
        "length_max": args.max_length,
        "breaks": breaks,
        "centers": centers,
        "inv_scales": inv_scales,
        "samples_per_axis": args.samples,
        "degree": args.degree,
        "powers": _powers(args.degree),
        "max_scaled_error": MAX_SCALED_ERROR,
        "state_order": STATE_ORDER,
        "output_order": OUTPUT_ORDER,
        "length_input": "target_L0_l, target_L0_r",
        "control_law": "U = K @ (expect_state - real_state)",
    }
    print(f"实际使用的模型：{config['model_path']}")
    print(
        f"Q={q_diag}\nR={r_diag}\n完整拟合范围：{breaks[0]}～{breaks[-1]} m", flush=True
    )
    coeffs, validation = fit_coefficients(model, config)
    export_c_library(coeffs, config, validation, args.output)
