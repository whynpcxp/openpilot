#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5677575667043581024) {
   out_5677575667043581024[0] = delta_x[0] + nom_x[0];
   out_5677575667043581024[1] = delta_x[1] + nom_x[1];
   out_5677575667043581024[2] = delta_x[2] + nom_x[2];
   out_5677575667043581024[3] = delta_x[3] + nom_x[3];
   out_5677575667043581024[4] = delta_x[4] + nom_x[4];
   out_5677575667043581024[5] = delta_x[5] + nom_x[5];
   out_5677575667043581024[6] = delta_x[6] + nom_x[6];
   out_5677575667043581024[7] = delta_x[7] + nom_x[7];
   out_5677575667043581024[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4285828498726984510) {
   out_4285828498726984510[0] = -nom_x[0] + true_x[0];
   out_4285828498726984510[1] = -nom_x[1] + true_x[1];
   out_4285828498726984510[2] = -nom_x[2] + true_x[2];
   out_4285828498726984510[3] = -nom_x[3] + true_x[3];
   out_4285828498726984510[4] = -nom_x[4] + true_x[4];
   out_4285828498726984510[5] = -nom_x[5] + true_x[5];
   out_4285828498726984510[6] = -nom_x[6] + true_x[6];
   out_4285828498726984510[7] = -nom_x[7] + true_x[7];
   out_4285828498726984510[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8348810779655370095) {
   out_8348810779655370095[0] = 1.0;
   out_8348810779655370095[1] = 0;
   out_8348810779655370095[2] = 0;
   out_8348810779655370095[3] = 0;
   out_8348810779655370095[4] = 0;
   out_8348810779655370095[5] = 0;
   out_8348810779655370095[6] = 0;
   out_8348810779655370095[7] = 0;
   out_8348810779655370095[8] = 0;
   out_8348810779655370095[9] = 0;
   out_8348810779655370095[10] = 1.0;
   out_8348810779655370095[11] = 0;
   out_8348810779655370095[12] = 0;
   out_8348810779655370095[13] = 0;
   out_8348810779655370095[14] = 0;
   out_8348810779655370095[15] = 0;
   out_8348810779655370095[16] = 0;
   out_8348810779655370095[17] = 0;
   out_8348810779655370095[18] = 0;
   out_8348810779655370095[19] = 0;
   out_8348810779655370095[20] = 1.0;
   out_8348810779655370095[21] = 0;
   out_8348810779655370095[22] = 0;
   out_8348810779655370095[23] = 0;
   out_8348810779655370095[24] = 0;
   out_8348810779655370095[25] = 0;
   out_8348810779655370095[26] = 0;
   out_8348810779655370095[27] = 0;
   out_8348810779655370095[28] = 0;
   out_8348810779655370095[29] = 0;
   out_8348810779655370095[30] = 1.0;
   out_8348810779655370095[31] = 0;
   out_8348810779655370095[32] = 0;
   out_8348810779655370095[33] = 0;
   out_8348810779655370095[34] = 0;
   out_8348810779655370095[35] = 0;
   out_8348810779655370095[36] = 0;
   out_8348810779655370095[37] = 0;
   out_8348810779655370095[38] = 0;
   out_8348810779655370095[39] = 0;
   out_8348810779655370095[40] = 1.0;
   out_8348810779655370095[41] = 0;
   out_8348810779655370095[42] = 0;
   out_8348810779655370095[43] = 0;
   out_8348810779655370095[44] = 0;
   out_8348810779655370095[45] = 0;
   out_8348810779655370095[46] = 0;
   out_8348810779655370095[47] = 0;
   out_8348810779655370095[48] = 0;
   out_8348810779655370095[49] = 0;
   out_8348810779655370095[50] = 1.0;
   out_8348810779655370095[51] = 0;
   out_8348810779655370095[52] = 0;
   out_8348810779655370095[53] = 0;
   out_8348810779655370095[54] = 0;
   out_8348810779655370095[55] = 0;
   out_8348810779655370095[56] = 0;
   out_8348810779655370095[57] = 0;
   out_8348810779655370095[58] = 0;
   out_8348810779655370095[59] = 0;
   out_8348810779655370095[60] = 1.0;
   out_8348810779655370095[61] = 0;
   out_8348810779655370095[62] = 0;
   out_8348810779655370095[63] = 0;
   out_8348810779655370095[64] = 0;
   out_8348810779655370095[65] = 0;
   out_8348810779655370095[66] = 0;
   out_8348810779655370095[67] = 0;
   out_8348810779655370095[68] = 0;
   out_8348810779655370095[69] = 0;
   out_8348810779655370095[70] = 1.0;
   out_8348810779655370095[71] = 0;
   out_8348810779655370095[72] = 0;
   out_8348810779655370095[73] = 0;
   out_8348810779655370095[74] = 0;
   out_8348810779655370095[75] = 0;
   out_8348810779655370095[76] = 0;
   out_8348810779655370095[77] = 0;
   out_8348810779655370095[78] = 0;
   out_8348810779655370095[79] = 0;
   out_8348810779655370095[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7345967355590271014) {
   out_7345967355590271014[0] = state[0];
   out_7345967355590271014[1] = state[1];
   out_7345967355590271014[2] = state[2];
   out_7345967355590271014[3] = state[3];
   out_7345967355590271014[4] = state[4];
   out_7345967355590271014[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7345967355590271014[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7345967355590271014[7] = state[7];
   out_7345967355590271014[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4489943646957803545) {
   out_4489943646957803545[0] = 1;
   out_4489943646957803545[1] = 0;
   out_4489943646957803545[2] = 0;
   out_4489943646957803545[3] = 0;
   out_4489943646957803545[4] = 0;
   out_4489943646957803545[5] = 0;
   out_4489943646957803545[6] = 0;
   out_4489943646957803545[7] = 0;
   out_4489943646957803545[8] = 0;
   out_4489943646957803545[9] = 0;
   out_4489943646957803545[10] = 1;
   out_4489943646957803545[11] = 0;
   out_4489943646957803545[12] = 0;
   out_4489943646957803545[13] = 0;
   out_4489943646957803545[14] = 0;
   out_4489943646957803545[15] = 0;
   out_4489943646957803545[16] = 0;
   out_4489943646957803545[17] = 0;
   out_4489943646957803545[18] = 0;
   out_4489943646957803545[19] = 0;
   out_4489943646957803545[20] = 1;
   out_4489943646957803545[21] = 0;
   out_4489943646957803545[22] = 0;
   out_4489943646957803545[23] = 0;
   out_4489943646957803545[24] = 0;
   out_4489943646957803545[25] = 0;
   out_4489943646957803545[26] = 0;
   out_4489943646957803545[27] = 0;
   out_4489943646957803545[28] = 0;
   out_4489943646957803545[29] = 0;
   out_4489943646957803545[30] = 1;
   out_4489943646957803545[31] = 0;
   out_4489943646957803545[32] = 0;
   out_4489943646957803545[33] = 0;
   out_4489943646957803545[34] = 0;
   out_4489943646957803545[35] = 0;
   out_4489943646957803545[36] = 0;
   out_4489943646957803545[37] = 0;
   out_4489943646957803545[38] = 0;
   out_4489943646957803545[39] = 0;
   out_4489943646957803545[40] = 1;
   out_4489943646957803545[41] = 0;
   out_4489943646957803545[42] = 0;
   out_4489943646957803545[43] = 0;
   out_4489943646957803545[44] = 0;
   out_4489943646957803545[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4489943646957803545[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4489943646957803545[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943646957803545[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943646957803545[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4489943646957803545[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4489943646957803545[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4489943646957803545[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4489943646957803545[53] = -9.8000000000000007*dt;
   out_4489943646957803545[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4489943646957803545[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4489943646957803545[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943646957803545[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943646957803545[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4489943646957803545[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4489943646957803545[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4489943646957803545[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4489943646957803545[62] = 0;
   out_4489943646957803545[63] = 0;
   out_4489943646957803545[64] = 0;
   out_4489943646957803545[65] = 0;
   out_4489943646957803545[66] = 0;
   out_4489943646957803545[67] = 0;
   out_4489943646957803545[68] = 0;
   out_4489943646957803545[69] = 0;
   out_4489943646957803545[70] = 1;
   out_4489943646957803545[71] = 0;
   out_4489943646957803545[72] = 0;
   out_4489943646957803545[73] = 0;
   out_4489943646957803545[74] = 0;
   out_4489943646957803545[75] = 0;
   out_4489943646957803545[76] = 0;
   out_4489943646957803545[77] = 0;
   out_4489943646957803545[78] = 0;
   out_4489943646957803545[79] = 0;
   out_4489943646957803545[80] = 1;
}
void h_25(double *state, double *unused, double *out_3232475677556297302) {
   out_3232475677556297302[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5855662715343251547) {
   out_5855662715343251547[0] = 0;
   out_5855662715343251547[1] = 0;
   out_5855662715343251547[2] = 0;
   out_5855662715343251547[3] = 0;
   out_5855662715343251547[4] = 0;
   out_5855662715343251547[5] = 0;
   out_5855662715343251547[6] = 1;
   out_5855662715343251547[7] = 0;
   out_5855662715343251547[8] = 0;
}
void h_24(double *state, double *unused, double *out_8858318705719087039) {
   out_8858318705719087039[0] = state[4];
   out_8858318705719087039[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4871596844501937879) {
   out_4871596844501937879[0] = 0;
   out_4871596844501937879[1] = 0;
   out_4871596844501937879[2] = 0;
   out_4871596844501937879[3] = 0;
   out_4871596844501937879[4] = 1;
   out_4871596844501937879[5] = 0;
   out_4871596844501937879[6] = 0;
   out_4871596844501937879[7] = 0;
   out_4871596844501937879[8] = 0;
   out_4871596844501937879[9] = 0;
   out_4871596844501937879[10] = 0;
   out_4871596844501937879[11] = 0;
   out_4871596844501937879[12] = 0;
   out_4871596844501937879[13] = 0;
   out_4871596844501937879[14] = 1;
   out_4871596844501937879[15] = 0;
   out_4871596844501937879[16] = 0;
   out_4871596844501937879[17] = 0;
}
void h_30(double *state, double *unused, double *out_1020361881207415855) {
   out_1020361881207415855[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5985001662486491617) {
   out_5985001662486491617[0] = 0;
   out_5985001662486491617[1] = 0;
   out_5985001662486491617[2] = 0;
   out_5985001662486491617[3] = 0;
   out_5985001662486491617[4] = 1;
   out_5985001662486491617[5] = 0;
   out_5985001662486491617[6] = 0;
   out_5985001662486491617[7] = 0;
   out_5985001662486491617[8] = 0;
}
void h_26(double *state, double *unused, double *out_6022035148868261500) {
   out_6022035148868261500[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8849578039492243845) {
   out_8849578039492243845[0] = 0;
   out_8849578039492243845[1] = 0;
   out_8849578039492243845[2] = 0;
   out_8849578039492243845[3] = 0;
   out_8849578039492243845[4] = 0;
   out_8849578039492243845[5] = 0;
   out_8849578039492243845[6] = 0;
   out_8849578039492243845[7] = 1;
   out_8849578039492243845[8] = 0;
}
void h_27(double *state, double *unused, double *out_8551699660585373453) {
   out_8551699660585373453[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8159764974286916528) {
   out_8159764974286916528[0] = 0;
   out_8159764974286916528[1] = 0;
   out_8159764974286916528[2] = 0;
   out_8159764974286916528[3] = 1;
   out_8159764974286916528[4] = 0;
   out_8159764974286916528[5] = 0;
   out_8159764974286916528[6] = 0;
   out_8159764974286916528[7] = 0;
   out_8159764974286916528[8] = 0;
}
void h_29(double *state, double *unused, double *out_3624767202016113506) {
   out_3624767202016113506[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8573616372553084055) {
   out_8573616372553084055[0] = 0;
   out_8573616372553084055[1] = 1;
   out_8573616372553084055[2] = 0;
   out_8573616372553084055[3] = 0;
   out_8573616372553084055[4] = 0;
   out_8573616372553084055[5] = 0;
   out_8573616372553084055[6] = 0;
   out_8573616372553084055[7] = 0;
   out_8573616372553084055[8] = 0;
}
void h_28(double *state, double *unused, double *out_2892281709331112384) {
   out_2892281709331112384[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7909497429591141310) {
   out_7909497429591141310[0] = 1;
   out_7909497429591141310[1] = 0;
   out_7909497429591141310[2] = 0;
   out_7909497429591141310[3] = 0;
   out_7909497429591141310[4] = 0;
   out_7909497429591141310[5] = 0;
   out_7909497429591141310[6] = 0;
   out_7909497429591141310[7] = 0;
   out_7909497429591141310[8] = 0;
}
void h_31(double *state, double *unused, double *out_7451003139332581873) {
   out_7451003139332581873[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5825016753466291119) {
   out_5825016753466291119[0] = 0;
   out_5825016753466291119[1] = 0;
   out_5825016753466291119[2] = 0;
   out_5825016753466291119[3] = 0;
   out_5825016753466291119[4] = 0;
   out_5825016753466291119[5] = 0;
   out_5825016753466291119[6] = 0;
   out_5825016753466291119[7] = 0;
   out_5825016753466291119[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5677575667043581024) {
  err_fun(nom_x, delta_x, out_5677575667043581024);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4285828498726984510) {
  inv_err_fun(nom_x, true_x, out_4285828498726984510);
}
void car_H_mod_fun(double *state, double *out_8348810779655370095) {
  H_mod_fun(state, out_8348810779655370095);
}
void car_f_fun(double *state, double dt, double *out_7345967355590271014) {
  f_fun(state,  dt, out_7345967355590271014);
}
void car_F_fun(double *state, double dt, double *out_4489943646957803545) {
  F_fun(state,  dt, out_4489943646957803545);
}
void car_h_25(double *state, double *unused, double *out_3232475677556297302) {
  h_25(state, unused, out_3232475677556297302);
}
void car_H_25(double *state, double *unused, double *out_5855662715343251547) {
  H_25(state, unused, out_5855662715343251547);
}
void car_h_24(double *state, double *unused, double *out_8858318705719087039) {
  h_24(state, unused, out_8858318705719087039);
}
void car_H_24(double *state, double *unused, double *out_4871596844501937879) {
  H_24(state, unused, out_4871596844501937879);
}
void car_h_30(double *state, double *unused, double *out_1020361881207415855) {
  h_30(state, unused, out_1020361881207415855);
}
void car_H_30(double *state, double *unused, double *out_5985001662486491617) {
  H_30(state, unused, out_5985001662486491617);
}
void car_h_26(double *state, double *unused, double *out_6022035148868261500) {
  h_26(state, unused, out_6022035148868261500);
}
void car_H_26(double *state, double *unused, double *out_8849578039492243845) {
  H_26(state, unused, out_8849578039492243845);
}
void car_h_27(double *state, double *unused, double *out_8551699660585373453) {
  h_27(state, unused, out_8551699660585373453);
}
void car_H_27(double *state, double *unused, double *out_8159764974286916528) {
  H_27(state, unused, out_8159764974286916528);
}
void car_h_29(double *state, double *unused, double *out_3624767202016113506) {
  h_29(state, unused, out_3624767202016113506);
}
void car_H_29(double *state, double *unused, double *out_8573616372553084055) {
  H_29(state, unused, out_8573616372553084055);
}
void car_h_28(double *state, double *unused, double *out_2892281709331112384) {
  h_28(state, unused, out_2892281709331112384);
}
void car_H_28(double *state, double *unused, double *out_7909497429591141310) {
  H_28(state, unused, out_7909497429591141310);
}
void car_h_31(double *state, double *unused, double *out_7451003139332581873) {
  h_31(state, unused, out_7451003139332581873);
}
void car_H_31(double *state, double *unused, double *out_5825016753466291119) {
  H_31(state, unused, out_5825016753466291119);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
