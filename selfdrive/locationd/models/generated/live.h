#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6327026773264915096);
void live_err_fun(double *nom_x, double *delta_x, double *out_8067863775592439168);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_357558411111015608);
void live_H_mod_fun(double *state, double *out_5377985231292781310);
void live_f_fun(double *state, double dt, double *out_5336262391330574405);
void live_F_fun(double *state, double dt, double *out_5727986461689315061);
void live_h_4(double *state, double *unused, double *out_7307035997982969824);
void live_H_4(double *state, double *unused, double *out_6677252187169356223);
void live_h_9(double *state, double *unused, double *out_7482255631790547869);
void live_H_9(double *state, double *unused, double *out_6918441833798946868);
void live_h_10(double *state, double *unused, double *out_1748597886089026286);
void live_H_10(double *state, double *unused, double *out_7519167040950150340);
void live_h_12(double *state, double *unused, double *out_1491269639460264899);
void live_H_12(double *state, double *unused, double *out_6750035478508233598);
void live_h_35(double *state, double *unused, double *out_8419097311840231893);
void live_H_35(double *state, double *unused, double *out_4004472446183219889);
void live_h_32(double *state, double *unused, double *out_8020283997342969519);
void live_H_32(double *state, double *unused, double *out_1796831926024392669);
void live_h_13(double *state, double *unused, double *out_3300464686285788565);
void live_H_13(double *state, double *unused, double *out_7803543419208219348);
void live_h_14(double *state, double *unused, double *out_7482255631790547869);
void live_H_14(double *state, double *unused, double *out_6918441833798946868);
void live_h_33(double *state, double *unused, double *out_306515413301644296);
void live_H_33(double *state, double *unused, double *out_853915441544362285);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}