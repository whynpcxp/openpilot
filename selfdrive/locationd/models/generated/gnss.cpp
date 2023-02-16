#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7273923111829202944) {
   out_7273923111829202944[0] = delta_x[0] + nom_x[0];
   out_7273923111829202944[1] = delta_x[1] + nom_x[1];
   out_7273923111829202944[2] = delta_x[2] + nom_x[2];
   out_7273923111829202944[3] = delta_x[3] + nom_x[3];
   out_7273923111829202944[4] = delta_x[4] + nom_x[4];
   out_7273923111829202944[5] = delta_x[5] + nom_x[5];
   out_7273923111829202944[6] = delta_x[6] + nom_x[6];
   out_7273923111829202944[7] = delta_x[7] + nom_x[7];
   out_7273923111829202944[8] = delta_x[8] + nom_x[8];
   out_7273923111829202944[9] = delta_x[9] + nom_x[9];
   out_7273923111829202944[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7905786940587894555) {
   out_7905786940587894555[0] = -nom_x[0] + true_x[0];
   out_7905786940587894555[1] = -nom_x[1] + true_x[1];
   out_7905786940587894555[2] = -nom_x[2] + true_x[2];
   out_7905786940587894555[3] = -nom_x[3] + true_x[3];
   out_7905786940587894555[4] = -nom_x[4] + true_x[4];
   out_7905786940587894555[5] = -nom_x[5] + true_x[5];
   out_7905786940587894555[6] = -nom_x[6] + true_x[6];
   out_7905786940587894555[7] = -nom_x[7] + true_x[7];
   out_7905786940587894555[8] = -nom_x[8] + true_x[8];
   out_7905786940587894555[9] = -nom_x[9] + true_x[9];
   out_7905786940587894555[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1620911765878707155) {
   out_1620911765878707155[0] = 1.0;
   out_1620911765878707155[1] = 0;
   out_1620911765878707155[2] = 0;
   out_1620911765878707155[3] = 0;
   out_1620911765878707155[4] = 0;
   out_1620911765878707155[5] = 0;
   out_1620911765878707155[6] = 0;
   out_1620911765878707155[7] = 0;
   out_1620911765878707155[8] = 0;
   out_1620911765878707155[9] = 0;
   out_1620911765878707155[10] = 0;
   out_1620911765878707155[11] = 0;
   out_1620911765878707155[12] = 1.0;
   out_1620911765878707155[13] = 0;
   out_1620911765878707155[14] = 0;
   out_1620911765878707155[15] = 0;
   out_1620911765878707155[16] = 0;
   out_1620911765878707155[17] = 0;
   out_1620911765878707155[18] = 0;
   out_1620911765878707155[19] = 0;
   out_1620911765878707155[20] = 0;
   out_1620911765878707155[21] = 0;
   out_1620911765878707155[22] = 0;
   out_1620911765878707155[23] = 0;
   out_1620911765878707155[24] = 1.0;
   out_1620911765878707155[25] = 0;
   out_1620911765878707155[26] = 0;
   out_1620911765878707155[27] = 0;
   out_1620911765878707155[28] = 0;
   out_1620911765878707155[29] = 0;
   out_1620911765878707155[30] = 0;
   out_1620911765878707155[31] = 0;
   out_1620911765878707155[32] = 0;
   out_1620911765878707155[33] = 0;
   out_1620911765878707155[34] = 0;
   out_1620911765878707155[35] = 0;
   out_1620911765878707155[36] = 1.0;
   out_1620911765878707155[37] = 0;
   out_1620911765878707155[38] = 0;
   out_1620911765878707155[39] = 0;
   out_1620911765878707155[40] = 0;
   out_1620911765878707155[41] = 0;
   out_1620911765878707155[42] = 0;
   out_1620911765878707155[43] = 0;
   out_1620911765878707155[44] = 0;
   out_1620911765878707155[45] = 0;
   out_1620911765878707155[46] = 0;
   out_1620911765878707155[47] = 0;
   out_1620911765878707155[48] = 1.0;
   out_1620911765878707155[49] = 0;
   out_1620911765878707155[50] = 0;
   out_1620911765878707155[51] = 0;
   out_1620911765878707155[52] = 0;
   out_1620911765878707155[53] = 0;
   out_1620911765878707155[54] = 0;
   out_1620911765878707155[55] = 0;
   out_1620911765878707155[56] = 0;
   out_1620911765878707155[57] = 0;
   out_1620911765878707155[58] = 0;
   out_1620911765878707155[59] = 0;
   out_1620911765878707155[60] = 1.0;
   out_1620911765878707155[61] = 0;
   out_1620911765878707155[62] = 0;
   out_1620911765878707155[63] = 0;
   out_1620911765878707155[64] = 0;
   out_1620911765878707155[65] = 0;
   out_1620911765878707155[66] = 0;
   out_1620911765878707155[67] = 0;
   out_1620911765878707155[68] = 0;
   out_1620911765878707155[69] = 0;
   out_1620911765878707155[70] = 0;
   out_1620911765878707155[71] = 0;
   out_1620911765878707155[72] = 1.0;
   out_1620911765878707155[73] = 0;
   out_1620911765878707155[74] = 0;
   out_1620911765878707155[75] = 0;
   out_1620911765878707155[76] = 0;
   out_1620911765878707155[77] = 0;
   out_1620911765878707155[78] = 0;
   out_1620911765878707155[79] = 0;
   out_1620911765878707155[80] = 0;
   out_1620911765878707155[81] = 0;
   out_1620911765878707155[82] = 0;
   out_1620911765878707155[83] = 0;
   out_1620911765878707155[84] = 1.0;
   out_1620911765878707155[85] = 0;
   out_1620911765878707155[86] = 0;
   out_1620911765878707155[87] = 0;
   out_1620911765878707155[88] = 0;
   out_1620911765878707155[89] = 0;
   out_1620911765878707155[90] = 0;
   out_1620911765878707155[91] = 0;
   out_1620911765878707155[92] = 0;
   out_1620911765878707155[93] = 0;
   out_1620911765878707155[94] = 0;
   out_1620911765878707155[95] = 0;
   out_1620911765878707155[96] = 1.0;
   out_1620911765878707155[97] = 0;
   out_1620911765878707155[98] = 0;
   out_1620911765878707155[99] = 0;
   out_1620911765878707155[100] = 0;
   out_1620911765878707155[101] = 0;
   out_1620911765878707155[102] = 0;
   out_1620911765878707155[103] = 0;
   out_1620911765878707155[104] = 0;
   out_1620911765878707155[105] = 0;
   out_1620911765878707155[106] = 0;
   out_1620911765878707155[107] = 0;
   out_1620911765878707155[108] = 1.0;
   out_1620911765878707155[109] = 0;
   out_1620911765878707155[110] = 0;
   out_1620911765878707155[111] = 0;
   out_1620911765878707155[112] = 0;
   out_1620911765878707155[113] = 0;
   out_1620911765878707155[114] = 0;
   out_1620911765878707155[115] = 0;
   out_1620911765878707155[116] = 0;
   out_1620911765878707155[117] = 0;
   out_1620911765878707155[118] = 0;
   out_1620911765878707155[119] = 0;
   out_1620911765878707155[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_1109934104538259817) {
   out_1109934104538259817[0] = dt*state[3] + state[0];
   out_1109934104538259817[1] = dt*state[4] + state[1];
   out_1109934104538259817[2] = dt*state[5] + state[2];
   out_1109934104538259817[3] = state[3];
   out_1109934104538259817[4] = state[4];
   out_1109934104538259817[5] = state[5];
   out_1109934104538259817[6] = dt*state[7] + state[6];
   out_1109934104538259817[7] = dt*state[8] + state[7];
   out_1109934104538259817[8] = state[8];
   out_1109934104538259817[9] = state[9];
   out_1109934104538259817[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2778871465064819331) {
   out_2778871465064819331[0] = 1;
   out_2778871465064819331[1] = 0;
   out_2778871465064819331[2] = 0;
   out_2778871465064819331[3] = dt;
   out_2778871465064819331[4] = 0;
   out_2778871465064819331[5] = 0;
   out_2778871465064819331[6] = 0;
   out_2778871465064819331[7] = 0;
   out_2778871465064819331[8] = 0;
   out_2778871465064819331[9] = 0;
   out_2778871465064819331[10] = 0;
   out_2778871465064819331[11] = 0;
   out_2778871465064819331[12] = 1;
   out_2778871465064819331[13] = 0;
   out_2778871465064819331[14] = 0;
   out_2778871465064819331[15] = dt;
   out_2778871465064819331[16] = 0;
   out_2778871465064819331[17] = 0;
   out_2778871465064819331[18] = 0;
   out_2778871465064819331[19] = 0;
   out_2778871465064819331[20] = 0;
   out_2778871465064819331[21] = 0;
   out_2778871465064819331[22] = 0;
   out_2778871465064819331[23] = 0;
   out_2778871465064819331[24] = 1;
   out_2778871465064819331[25] = 0;
   out_2778871465064819331[26] = 0;
   out_2778871465064819331[27] = dt;
   out_2778871465064819331[28] = 0;
   out_2778871465064819331[29] = 0;
   out_2778871465064819331[30] = 0;
   out_2778871465064819331[31] = 0;
   out_2778871465064819331[32] = 0;
   out_2778871465064819331[33] = 0;
   out_2778871465064819331[34] = 0;
   out_2778871465064819331[35] = 0;
   out_2778871465064819331[36] = 1;
   out_2778871465064819331[37] = 0;
   out_2778871465064819331[38] = 0;
   out_2778871465064819331[39] = 0;
   out_2778871465064819331[40] = 0;
   out_2778871465064819331[41] = 0;
   out_2778871465064819331[42] = 0;
   out_2778871465064819331[43] = 0;
   out_2778871465064819331[44] = 0;
   out_2778871465064819331[45] = 0;
   out_2778871465064819331[46] = 0;
   out_2778871465064819331[47] = 0;
   out_2778871465064819331[48] = 1;
   out_2778871465064819331[49] = 0;
   out_2778871465064819331[50] = 0;
   out_2778871465064819331[51] = 0;
   out_2778871465064819331[52] = 0;
   out_2778871465064819331[53] = 0;
   out_2778871465064819331[54] = 0;
   out_2778871465064819331[55] = 0;
   out_2778871465064819331[56] = 0;
   out_2778871465064819331[57] = 0;
   out_2778871465064819331[58] = 0;
   out_2778871465064819331[59] = 0;
   out_2778871465064819331[60] = 1;
   out_2778871465064819331[61] = 0;
   out_2778871465064819331[62] = 0;
   out_2778871465064819331[63] = 0;
   out_2778871465064819331[64] = 0;
   out_2778871465064819331[65] = 0;
   out_2778871465064819331[66] = 0;
   out_2778871465064819331[67] = 0;
   out_2778871465064819331[68] = 0;
   out_2778871465064819331[69] = 0;
   out_2778871465064819331[70] = 0;
   out_2778871465064819331[71] = 0;
   out_2778871465064819331[72] = 1;
   out_2778871465064819331[73] = dt;
   out_2778871465064819331[74] = 0;
   out_2778871465064819331[75] = 0;
   out_2778871465064819331[76] = 0;
   out_2778871465064819331[77] = 0;
   out_2778871465064819331[78] = 0;
   out_2778871465064819331[79] = 0;
   out_2778871465064819331[80] = 0;
   out_2778871465064819331[81] = 0;
   out_2778871465064819331[82] = 0;
   out_2778871465064819331[83] = 0;
   out_2778871465064819331[84] = 1;
   out_2778871465064819331[85] = dt;
   out_2778871465064819331[86] = 0;
   out_2778871465064819331[87] = 0;
   out_2778871465064819331[88] = 0;
   out_2778871465064819331[89] = 0;
   out_2778871465064819331[90] = 0;
   out_2778871465064819331[91] = 0;
   out_2778871465064819331[92] = 0;
   out_2778871465064819331[93] = 0;
   out_2778871465064819331[94] = 0;
   out_2778871465064819331[95] = 0;
   out_2778871465064819331[96] = 1;
   out_2778871465064819331[97] = 0;
   out_2778871465064819331[98] = 0;
   out_2778871465064819331[99] = 0;
   out_2778871465064819331[100] = 0;
   out_2778871465064819331[101] = 0;
   out_2778871465064819331[102] = 0;
   out_2778871465064819331[103] = 0;
   out_2778871465064819331[104] = 0;
   out_2778871465064819331[105] = 0;
   out_2778871465064819331[106] = 0;
   out_2778871465064819331[107] = 0;
   out_2778871465064819331[108] = 1;
   out_2778871465064819331[109] = 0;
   out_2778871465064819331[110] = 0;
   out_2778871465064819331[111] = 0;
   out_2778871465064819331[112] = 0;
   out_2778871465064819331[113] = 0;
   out_2778871465064819331[114] = 0;
   out_2778871465064819331[115] = 0;
   out_2778871465064819331[116] = 0;
   out_2778871465064819331[117] = 0;
   out_2778871465064819331[118] = 0;
   out_2778871465064819331[119] = 0;
   out_2778871465064819331[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3873829254776754261) {
   out_3873829254776754261[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_718420411065544009) {
   out_718420411065544009[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_718420411065544009[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_718420411065544009[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_718420411065544009[3] = 0;
   out_718420411065544009[4] = 0;
   out_718420411065544009[5] = 0;
   out_718420411065544009[6] = 1;
   out_718420411065544009[7] = 0;
   out_718420411065544009[8] = 0;
   out_718420411065544009[9] = 0;
   out_718420411065544009[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4619391979817673807) {
   out_4619391979817673807[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_6207271946772719838) {
   out_6207271946772719838[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6207271946772719838[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6207271946772719838[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6207271946772719838[3] = 0;
   out_6207271946772719838[4] = 0;
   out_6207271946772719838[5] = 0;
   out_6207271946772719838[6] = 1;
   out_6207271946772719838[7] = 0;
   out_6207271946772719838[8] = 0;
   out_6207271946772719838[9] = 1;
   out_6207271946772719838[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3087197782599609122) {
   out_3087197782599609122[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_5479571354101766201) {
   out_5479571354101766201[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[6] = 0;
   out_5479571354101766201[7] = 1;
   out_5479571354101766201[8] = 0;
   out_5479571354101766201[9] = 0;
   out_5479571354101766201[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3087197782599609122) {
   out_3087197782599609122[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_5479571354101766201) {
   out_5479571354101766201[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_5479571354101766201[6] = 0;
   out_5479571354101766201[7] = 1;
   out_5479571354101766201[8] = 0;
   out_5479571354101766201[9] = 0;
   out_5479571354101766201[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7273923111829202944) {
  err_fun(nom_x, delta_x, out_7273923111829202944);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7905786940587894555) {
  inv_err_fun(nom_x, true_x, out_7905786940587894555);
}
void gnss_H_mod_fun(double *state, double *out_1620911765878707155) {
  H_mod_fun(state, out_1620911765878707155);
}
void gnss_f_fun(double *state, double dt, double *out_1109934104538259817) {
  f_fun(state,  dt, out_1109934104538259817);
}
void gnss_F_fun(double *state, double dt, double *out_2778871465064819331) {
  F_fun(state,  dt, out_2778871465064819331);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3873829254776754261) {
  h_6(state, sat_pos, out_3873829254776754261);
}
void gnss_H_6(double *state, double *sat_pos, double *out_718420411065544009) {
  H_6(state, sat_pos, out_718420411065544009);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4619391979817673807) {
  h_20(state, sat_pos, out_4619391979817673807);
}
void gnss_H_20(double *state, double *sat_pos, double *out_6207271946772719838) {
  H_20(state, sat_pos, out_6207271946772719838);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3087197782599609122) {
  h_7(state, sat_pos_vel, out_3087197782599609122);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_5479571354101766201) {
  H_7(state, sat_pos_vel, out_5479571354101766201);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3087197782599609122) {
  h_21(state, sat_pos_vel, out_3087197782599609122);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_5479571354101766201) {
  H_21(state, sat_pos_vel, out_5479571354101766201);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
