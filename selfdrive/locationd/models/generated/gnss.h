#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7273923111829202944);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7905786940587894555);
void gnss_H_mod_fun(double *state, double *out_1620911765878707155);
void gnss_f_fun(double *state, double dt, double *out_1109934104538259817);
void gnss_F_fun(double *state, double dt, double *out_2778871465064819331);
void gnss_h_6(double *state, double *sat_pos, double *out_3873829254776754261);
void gnss_H_6(double *state, double *sat_pos, double *out_718420411065544009);
void gnss_h_20(double *state, double *sat_pos, double *out_4619391979817673807);
void gnss_H_20(double *state, double *sat_pos, double *out_6207271946772719838);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3087197782599609122);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5479571354101766201);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3087197782599609122);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5479571354101766201);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}