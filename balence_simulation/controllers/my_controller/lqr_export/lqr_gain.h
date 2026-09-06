/* Generated from lagrange.K; model SHA256: b10d4e5f469e9c89f0fe5fd691dd85cd061a7846fab71e948c905910cec3cd07
 * Q: [1000, 5, 500, 1, 500, 1, 500, 1, 10000, 10]
 * R: [1, 1, 1, 1]
 * K is approximate; changing model or weights requires regeneration. */

#ifndef LQR_GAIN_H
#define LQR_GAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define LQR_STATE_COUNT 10
#define LQR_OUTPUT_COUNT 4
#define LQR_SEGMENT_COUNT 2
#define LQR_LENGTH_MIN 0.160002798f
#define LQR_LENGTH_MAX 0.389576942f

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
}
#endif
#endif
