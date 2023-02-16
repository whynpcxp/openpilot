#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5677575667043581024);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4285828498726984510);
void car_H_mod_fun(double *state, double *out_8348810779655370095);
void car_f_fun(double *state, double dt, double *out_7345967355590271014);
void car_F_fun(double *state, double dt, double *out_4489943646957803545);
void car_h_25(double *state, double *unused, double *out_3232475677556297302);
void car_H_25(double *state, double *unused, double *out_5855662715343251547);
void car_h_24(double *state, double *unused, double *out_8858318705719087039);
void car_H_24(double *state, double *unused, double *out_4871596844501937879);
void car_h_30(double *state, double *unused, double *out_1020361881207415855);
void car_H_30(double *state, double *unused, double *out_5985001662486491617);
void car_h_26(double *state, double *unused, double *out_6022035148868261500);
void car_H_26(double *state, double *unused, double *out_8849578039492243845);
void car_h_27(double *state, double *unused, double *out_8551699660585373453);
void car_H_27(double *state, double *unused, double *out_8159764974286916528);
void car_h_29(double *state, double *unused, double *out_3624767202016113506);
void car_H_29(double *state, double *unused, double *out_8573616372553084055);
void car_h_28(double *state, double *unused, double *out_2892281709331112384);
void car_H_28(double *state, double *unused, double *out_7909497429591141310);
void car_h_31(double *state, double *unused, double *out_7451003139332581873);
void car_H_31(double *state, double *unused, double *out_5825016753466291119);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}