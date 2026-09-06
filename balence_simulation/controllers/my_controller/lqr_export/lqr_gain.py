"""拟合版 lagrange.K：将本文件和 lqr_fit.json 放到控制器目录。"""
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
