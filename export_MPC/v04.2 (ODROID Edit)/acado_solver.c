/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 13];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 13 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 13 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 13 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 13 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 13 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 13 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 13 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 13 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 13 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 13 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 13 + 11];
acadoWorkspace.state[12] = acadoVariables.x[lRun1 * 13 + 12];

acadoWorkspace.state[234] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[235] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[236] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[237] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 13] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 13 + 13];
acadoWorkspace.d[lRun1 * 13 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 13 + 14];
acadoWorkspace.d[lRun1 * 13 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 13 + 15];
acadoWorkspace.d[lRun1 * 13 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 13 + 16];
acadoWorkspace.d[lRun1 * 13 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 13 + 17];
acadoWorkspace.d[lRun1 * 13 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 13 + 18];
acadoWorkspace.d[lRun1 * 13 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 13 + 19];
acadoWorkspace.d[lRun1 * 13 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 13 + 20];
acadoWorkspace.d[lRun1 * 13 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 13 + 21];
acadoWorkspace.d[lRun1 * 13 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 13 + 22];
acadoWorkspace.d[lRun1 * 13 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 13 + 23];
acadoWorkspace.d[lRun1 * 13 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 13 + 24];
acadoWorkspace.d[lRun1 * 13 + 12] = acadoWorkspace.state[12] - acadoVariables.x[lRun1 * 13 + 25];

for (lRun2 = 0; lRun2 < 169; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 169))] = acadoWorkspace.state[lRun2 + 13];


acadoWorkspace.evGu[lRun1 * 52] = acadoWorkspace.state[182];
acadoWorkspace.evGu[lRun1 * 52 + 1] = acadoWorkspace.state[183];
acadoWorkspace.evGu[lRun1 * 52 + 2] = acadoWorkspace.state[184];
acadoWorkspace.evGu[lRun1 * 52 + 3] = acadoWorkspace.state[185];
acadoWorkspace.evGu[lRun1 * 52 + 4] = acadoWorkspace.state[186];
acadoWorkspace.evGu[lRun1 * 52 + 5] = acadoWorkspace.state[187];
acadoWorkspace.evGu[lRun1 * 52 + 6] = acadoWorkspace.state[188];
acadoWorkspace.evGu[lRun1 * 52 + 7] = acadoWorkspace.state[189];
acadoWorkspace.evGu[lRun1 * 52 + 8] = acadoWorkspace.state[190];
acadoWorkspace.evGu[lRun1 * 52 + 9] = acadoWorkspace.state[191];
acadoWorkspace.evGu[lRun1 * 52 + 10] = acadoWorkspace.state[192];
acadoWorkspace.evGu[lRun1 * 52 + 11] = acadoWorkspace.state[193];
acadoWorkspace.evGu[lRun1 * 52 + 12] = acadoWorkspace.state[194];
acadoWorkspace.evGu[lRun1 * 52 + 13] = acadoWorkspace.state[195];
acadoWorkspace.evGu[lRun1 * 52 + 14] = acadoWorkspace.state[196];
acadoWorkspace.evGu[lRun1 * 52 + 15] = acadoWorkspace.state[197];
acadoWorkspace.evGu[lRun1 * 52 + 16] = acadoWorkspace.state[198];
acadoWorkspace.evGu[lRun1 * 52 + 17] = acadoWorkspace.state[199];
acadoWorkspace.evGu[lRun1 * 52 + 18] = acadoWorkspace.state[200];
acadoWorkspace.evGu[lRun1 * 52 + 19] = acadoWorkspace.state[201];
acadoWorkspace.evGu[lRun1 * 52 + 20] = acadoWorkspace.state[202];
acadoWorkspace.evGu[lRun1 * 52 + 21] = acadoWorkspace.state[203];
acadoWorkspace.evGu[lRun1 * 52 + 22] = acadoWorkspace.state[204];
acadoWorkspace.evGu[lRun1 * 52 + 23] = acadoWorkspace.state[205];
acadoWorkspace.evGu[lRun1 * 52 + 24] = acadoWorkspace.state[206];
acadoWorkspace.evGu[lRun1 * 52 + 25] = acadoWorkspace.state[207];
acadoWorkspace.evGu[lRun1 * 52 + 26] = acadoWorkspace.state[208];
acadoWorkspace.evGu[lRun1 * 52 + 27] = acadoWorkspace.state[209];
acadoWorkspace.evGu[lRun1 * 52 + 28] = acadoWorkspace.state[210];
acadoWorkspace.evGu[lRun1 * 52 + 29] = acadoWorkspace.state[211];
acadoWorkspace.evGu[lRun1 * 52 + 30] = acadoWorkspace.state[212];
acadoWorkspace.evGu[lRun1 * 52 + 31] = acadoWorkspace.state[213];
acadoWorkspace.evGu[lRun1 * 52 + 32] = acadoWorkspace.state[214];
acadoWorkspace.evGu[lRun1 * 52 + 33] = acadoWorkspace.state[215];
acadoWorkspace.evGu[lRun1 * 52 + 34] = acadoWorkspace.state[216];
acadoWorkspace.evGu[lRun1 * 52 + 35] = acadoWorkspace.state[217];
acadoWorkspace.evGu[lRun1 * 52 + 36] = acadoWorkspace.state[218];
acadoWorkspace.evGu[lRun1 * 52 + 37] = acadoWorkspace.state[219];
acadoWorkspace.evGu[lRun1 * 52 + 38] = acadoWorkspace.state[220];
acadoWorkspace.evGu[lRun1 * 52 + 39] = acadoWorkspace.state[221];
acadoWorkspace.evGu[lRun1 * 52 + 40] = acadoWorkspace.state[222];
acadoWorkspace.evGu[lRun1 * 52 + 41] = acadoWorkspace.state[223];
acadoWorkspace.evGu[lRun1 * 52 + 42] = acadoWorkspace.state[224];
acadoWorkspace.evGu[lRun1 * 52 + 43] = acadoWorkspace.state[225];
acadoWorkspace.evGu[lRun1 * 52 + 44] = acadoWorkspace.state[226];
acadoWorkspace.evGu[lRun1 * 52 + 45] = acadoWorkspace.state[227];
acadoWorkspace.evGu[lRun1 * 52 + 46] = acadoWorkspace.state[228];
acadoWorkspace.evGu[lRun1 * 52 + 47] = acadoWorkspace.state[229];
acadoWorkspace.evGu[lRun1 * 52 + 48] = acadoWorkspace.state[230];
acadoWorkspace.evGu[lRun1 * 52 + 49] = acadoWorkspace.state[231];
acadoWorkspace.evGu[lRun1 * 52 + 50] = acadoWorkspace.state[232];
acadoWorkspace.evGu[lRun1 * 52 + 51] = acadoWorkspace.state[233];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 13;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[7];
out[7] = xd[8];
out[8] = xd[9];
out[9] = xd[10];
out[10] = xd[11];
out[11] = xd[12];
out[12] = u[0];
out[13] = u[1];
out[14] = u[2];
out[15] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[7];
out[7] = xd[8];
out[8] = xd[9];
out[9] = xd[10];
out[10] = xd[11];
out[11] = xd[12];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 13];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 13 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 13 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 13 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 13 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 13 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 13 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 13 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 13 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 13 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 13 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 13 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[runObj * 13 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[16] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 16] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 16 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 16 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 16 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 16 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 16 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 16 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 16 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 16 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 16 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 16 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 16 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 16 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 16 + 13] = acadoWorkspace.objValueOut[13];
acadoWorkspace.Dy[runObj * 16 + 14] = acadoWorkspace.objValueOut[14];
acadoWorkspace.Dy[runObj * 16 + 15] = acadoWorkspace.objValueOut[15];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[260];
acadoWorkspace.objValueIn[1] = acadoVariables.x[261];
acadoWorkspace.objValueIn[2] = acadoVariables.x[262];
acadoWorkspace.objValueIn[3] = acadoVariables.x[263];
acadoWorkspace.objValueIn[4] = acadoVariables.x[264];
acadoWorkspace.objValueIn[5] = acadoVariables.x[265];
acadoWorkspace.objValueIn[6] = acadoVariables.x[266];
acadoWorkspace.objValueIn[7] = acadoVariables.x[267];
acadoWorkspace.objValueIn[8] = acadoVariables.x[268];
acadoWorkspace.objValueIn[9] = acadoVariables.x[269];
acadoWorkspace.objValueIn[10] = acadoVariables.x[270];
acadoWorkspace.objValueIn[11] = acadoVariables.x[271];
acadoWorkspace.objValueIn[12] = acadoVariables.x[272];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11];

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44] + Gx1[12]*Gu1[48];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45] + Gx1[12]*Gu1[49];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46] + Gx1[12]*Gu1[50];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47] + Gx1[12]*Gu1[51];
Gu2[4] = + Gx1[13]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[8] + Gx1[16]*Gu1[12] + Gx1[17]*Gu1[16] + Gx1[18]*Gu1[20] + Gx1[19]*Gu1[24] + Gx1[20]*Gu1[28] + Gx1[21]*Gu1[32] + Gx1[22]*Gu1[36] + Gx1[23]*Gu1[40] + Gx1[24]*Gu1[44] + Gx1[25]*Gu1[48];
Gu2[5] = + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[9] + Gx1[16]*Gu1[13] + Gx1[17]*Gu1[17] + Gx1[18]*Gu1[21] + Gx1[19]*Gu1[25] + Gx1[20]*Gu1[29] + Gx1[21]*Gu1[33] + Gx1[22]*Gu1[37] + Gx1[23]*Gu1[41] + Gx1[24]*Gu1[45] + Gx1[25]*Gu1[49];
Gu2[6] = + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[15]*Gu1[10] + Gx1[16]*Gu1[14] + Gx1[17]*Gu1[18] + Gx1[18]*Gu1[22] + Gx1[19]*Gu1[26] + Gx1[20]*Gu1[30] + Gx1[21]*Gu1[34] + Gx1[22]*Gu1[38] + Gx1[23]*Gu1[42] + Gx1[24]*Gu1[46] + Gx1[25]*Gu1[50];
Gu2[7] = + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[15]*Gu1[11] + Gx1[16]*Gu1[15] + Gx1[17]*Gu1[19] + Gx1[18]*Gu1[23] + Gx1[19]*Gu1[27] + Gx1[20]*Gu1[31] + Gx1[21]*Gu1[35] + Gx1[22]*Gu1[39] + Gx1[23]*Gu1[43] + Gx1[24]*Gu1[47] + Gx1[25]*Gu1[51];
Gu2[8] = + Gx1[26]*Gu1[0] + Gx1[27]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[12] + Gx1[30]*Gu1[16] + Gx1[31]*Gu1[20] + Gx1[32]*Gu1[24] + Gx1[33]*Gu1[28] + Gx1[34]*Gu1[32] + Gx1[35]*Gu1[36] + Gx1[36]*Gu1[40] + Gx1[37]*Gu1[44] + Gx1[38]*Gu1[48];
Gu2[9] = + Gx1[26]*Gu1[1] + Gx1[27]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[13] + Gx1[30]*Gu1[17] + Gx1[31]*Gu1[21] + Gx1[32]*Gu1[25] + Gx1[33]*Gu1[29] + Gx1[34]*Gu1[33] + Gx1[35]*Gu1[37] + Gx1[36]*Gu1[41] + Gx1[37]*Gu1[45] + Gx1[38]*Gu1[49];
Gu2[10] = + Gx1[26]*Gu1[2] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[29]*Gu1[14] + Gx1[30]*Gu1[18] + Gx1[31]*Gu1[22] + Gx1[32]*Gu1[26] + Gx1[33]*Gu1[30] + Gx1[34]*Gu1[34] + Gx1[35]*Gu1[38] + Gx1[36]*Gu1[42] + Gx1[37]*Gu1[46] + Gx1[38]*Gu1[50];
Gu2[11] = + Gx1[26]*Gu1[3] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[29]*Gu1[15] + Gx1[30]*Gu1[19] + Gx1[31]*Gu1[23] + Gx1[32]*Gu1[27] + Gx1[33]*Gu1[31] + Gx1[34]*Gu1[35] + Gx1[35]*Gu1[39] + Gx1[36]*Gu1[43] + Gx1[37]*Gu1[47] + Gx1[38]*Gu1[51];
Gu2[12] = + Gx1[39]*Gu1[0] + Gx1[40]*Gu1[4] + Gx1[41]*Gu1[8] + Gx1[42]*Gu1[12] + Gx1[43]*Gu1[16] + Gx1[44]*Gu1[20] + Gx1[45]*Gu1[24] + Gx1[46]*Gu1[28] + Gx1[47]*Gu1[32] + Gx1[48]*Gu1[36] + Gx1[49]*Gu1[40] + Gx1[50]*Gu1[44] + Gx1[51]*Gu1[48];
Gu2[13] = + Gx1[39]*Gu1[1] + Gx1[40]*Gu1[5] + Gx1[41]*Gu1[9] + Gx1[42]*Gu1[13] + Gx1[43]*Gu1[17] + Gx1[44]*Gu1[21] + Gx1[45]*Gu1[25] + Gx1[46]*Gu1[29] + Gx1[47]*Gu1[33] + Gx1[48]*Gu1[37] + Gx1[49]*Gu1[41] + Gx1[50]*Gu1[45] + Gx1[51]*Gu1[49];
Gu2[14] = + Gx1[39]*Gu1[2] + Gx1[40]*Gu1[6] + Gx1[41]*Gu1[10] + Gx1[42]*Gu1[14] + Gx1[43]*Gu1[18] + Gx1[44]*Gu1[22] + Gx1[45]*Gu1[26] + Gx1[46]*Gu1[30] + Gx1[47]*Gu1[34] + Gx1[48]*Gu1[38] + Gx1[49]*Gu1[42] + Gx1[50]*Gu1[46] + Gx1[51]*Gu1[50];
Gu2[15] = + Gx1[39]*Gu1[3] + Gx1[40]*Gu1[7] + Gx1[41]*Gu1[11] + Gx1[42]*Gu1[15] + Gx1[43]*Gu1[19] + Gx1[44]*Gu1[23] + Gx1[45]*Gu1[27] + Gx1[46]*Gu1[31] + Gx1[47]*Gu1[35] + Gx1[48]*Gu1[39] + Gx1[49]*Gu1[43] + Gx1[50]*Gu1[47] + Gx1[51]*Gu1[51];
Gu2[16] = + Gx1[52]*Gu1[0] + Gx1[53]*Gu1[4] + Gx1[54]*Gu1[8] + Gx1[55]*Gu1[12] + Gx1[56]*Gu1[16] + Gx1[57]*Gu1[20] + Gx1[58]*Gu1[24] + Gx1[59]*Gu1[28] + Gx1[60]*Gu1[32] + Gx1[61]*Gu1[36] + Gx1[62]*Gu1[40] + Gx1[63]*Gu1[44] + Gx1[64]*Gu1[48];
Gu2[17] = + Gx1[52]*Gu1[1] + Gx1[53]*Gu1[5] + Gx1[54]*Gu1[9] + Gx1[55]*Gu1[13] + Gx1[56]*Gu1[17] + Gx1[57]*Gu1[21] + Gx1[58]*Gu1[25] + Gx1[59]*Gu1[29] + Gx1[60]*Gu1[33] + Gx1[61]*Gu1[37] + Gx1[62]*Gu1[41] + Gx1[63]*Gu1[45] + Gx1[64]*Gu1[49];
Gu2[18] = + Gx1[52]*Gu1[2] + Gx1[53]*Gu1[6] + Gx1[54]*Gu1[10] + Gx1[55]*Gu1[14] + Gx1[56]*Gu1[18] + Gx1[57]*Gu1[22] + Gx1[58]*Gu1[26] + Gx1[59]*Gu1[30] + Gx1[60]*Gu1[34] + Gx1[61]*Gu1[38] + Gx1[62]*Gu1[42] + Gx1[63]*Gu1[46] + Gx1[64]*Gu1[50];
Gu2[19] = + Gx1[52]*Gu1[3] + Gx1[53]*Gu1[7] + Gx1[54]*Gu1[11] + Gx1[55]*Gu1[15] + Gx1[56]*Gu1[19] + Gx1[57]*Gu1[23] + Gx1[58]*Gu1[27] + Gx1[59]*Gu1[31] + Gx1[60]*Gu1[35] + Gx1[61]*Gu1[39] + Gx1[62]*Gu1[43] + Gx1[63]*Gu1[47] + Gx1[64]*Gu1[51];
Gu2[20] = + Gx1[65]*Gu1[0] + Gx1[66]*Gu1[4] + Gx1[67]*Gu1[8] + Gx1[68]*Gu1[12] + Gx1[69]*Gu1[16] + Gx1[70]*Gu1[20] + Gx1[71]*Gu1[24] + Gx1[72]*Gu1[28] + Gx1[73]*Gu1[32] + Gx1[74]*Gu1[36] + Gx1[75]*Gu1[40] + Gx1[76]*Gu1[44] + Gx1[77]*Gu1[48];
Gu2[21] = + Gx1[65]*Gu1[1] + Gx1[66]*Gu1[5] + Gx1[67]*Gu1[9] + Gx1[68]*Gu1[13] + Gx1[69]*Gu1[17] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[25] + Gx1[72]*Gu1[29] + Gx1[73]*Gu1[33] + Gx1[74]*Gu1[37] + Gx1[75]*Gu1[41] + Gx1[76]*Gu1[45] + Gx1[77]*Gu1[49];
Gu2[22] = + Gx1[65]*Gu1[2] + Gx1[66]*Gu1[6] + Gx1[67]*Gu1[10] + Gx1[68]*Gu1[14] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[26] + Gx1[72]*Gu1[30] + Gx1[73]*Gu1[34] + Gx1[74]*Gu1[38] + Gx1[75]*Gu1[42] + Gx1[76]*Gu1[46] + Gx1[77]*Gu1[50];
Gu2[23] = + Gx1[65]*Gu1[3] + Gx1[66]*Gu1[7] + Gx1[67]*Gu1[11] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[27] + Gx1[72]*Gu1[31] + Gx1[73]*Gu1[35] + Gx1[74]*Gu1[39] + Gx1[75]*Gu1[43] + Gx1[76]*Gu1[47] + Gx1[77]*Gu1[51];
Gu2[24] = + Gx1[78]*Gu1[0] + Gx1[79]*Gu1[4] + Gx1[80]*Gu1[8] + Gx1[81]*Gu1[12] + Gx1[82]*Gu1[16] + Gx1[83]*Gu1[20] + Gx1[84]*Gu1[24] + Gx1[85]*Gu1[28] + Gx1[86]*Gu1[32] + Gx1[87]*Gu1[36] + Gx1[88]*Gu1[40] + Gx1[89]*Gu1[44] + Gx1[90]*Gu1[48];
Gu2[25] = + Gx1[78]*Gu1[1] + Gx1[79]*Gu1[5] + Gx1[80]*Gu1[9] + Gx1[81]*Gu1[13] + Gx1[82]*Gu1[17] + Gx1[83]*Gu1[21] + Gx1[84]*Gu1[25] + Gx1[85]*Gu1[29] + Gx1[86]*Gu1[33] + Gx1[87]*Gu1[37] + Gx1[88]*Gu1[41] + Gx1[89]*Gu1[45] + Gx1[90]*Gu1[49];
Gu2[26] = + Gx1[78]*Gu1[2] + Gx1[79]*Gu1[6] + Gx1[80]*Gu1[10] + Gx1[81]*Gu1[14] + Gx1[82]*Gu1[18] + Gx1[83]*Gu1[22] + Gx1[84]*Gu1[26] + Gx1[85]*Gu1[30] + Gx1[86]*Gu1[34] + Gx1[87]*Gu1[38] + Gx1[88]*Gu1[42] + Gx1[89]*Gu1[46] + Gx1[90]*Gu1[50];
Gu2[27] = + Gx1[78]*Gu1[3] + Gx1[79]*Gu1[7] + Gx1[80]*Gu1[11] + Gx1[81]*Gu1[15] + Gx1[82]*Gu1[19] + Gx1[83]*Gu1[23] + Gx1[84]*Gu1[27] + Gx1[85]*Gu1[31] + Gx1[86]*Gu1[35] + Gx1[87]*Gu1[39] + Gx1[88]*Gu1[43] + Gx1[89]*Gu1[47] + Gx1[90]*Gu1[51];
Gu2[28] = + Gx1[91]*Gu1[0] + Gx1[92]*Gu1[4] + Gx1[93]*Gu1[8] + Gx1[94]*Gu1[12] + Gx1[95]*Gu1[16] + Gx1[96]*Gu1[20] + Gx1[97]*Gu1[24] + Gx1[98]*Gu1[28] + Gx1[99]*Gu1[32] + Gx1[100]*Gu1[36] + Gx1[101]*Gu1[40] + Gx1[102]*Gu1[44] + Gx1[103]*Gu1[48];
Gu2[29] = + Gx1[91]*Gu1[1] + Gx1[92]*Gu1[5] + Gx1[93]*Gu1[9] + Gx1[94]*Gu1[13] + Gx1[95]*Gu1[17] + Gx1[96]*Gu1[21] + Gx1[97]*Gu1[25] + Gx1[98]*Gu1[29] + Gx1[99]*Gu1[33] + Gx1[100]*Gu1[37] + Gx1[101]*Gu1[41] + Gx1[102]*Gu1[45] + Gx1[103]*Gu1[49];
Gu2[30] = + Gx1[91]*Gu1[2] + Gx1[92]*Gu1[6] + Gx1[93]*Gu1[10] + Gx1[94]*Gu1[14] + Gx1[95]*Gu1[18] + Gx1[96]*Gu1[22] + Gx1[97]*Gu1[26] + Gx1[98]*Gu1[30] + Gx1[99]*Gu1[34] + Gx1[100]*Gu1[38] + Gx1[101]*Gu1[42] + Gx1[102]*Gu1[46] + Gx1[103]*Gu1[50];
Gu2[31] = + Gx1[91]*Gu1[3] + Gx1[92]*Gu1[7] + Gx1[93]*Gu1[11] + Gx1[94]*Gu1[15] + Gx1[95]*Gu1[19] + Gx1[96]*Gu1[23] + Gx1[97]*Gu1[27] + Gx1[98]*Gu1[31] + Gx1[99]*Gu1[35] + Gx1[100]*Gu1[39] + Gx1[101]*Gu1[43] + Gx1[102]*Gu1[47] + Gx1[103]*Gu1[51];
Gu2[32] = + Gx1[104]*Gu1[0] + Gx1[105]*Gu1[4] + Gx1[106]*Gu1[8] + Gx1[107]*Gu1[12] + Gx1[108]*Gu1[16] + Gx1[109]*Gu1[20] + Gx1[110]*Gu1[24] + Gx1[111]*Gu1[28] + Gx1[112]*Gu1[32] + Gx1[113]*Gu1[36] + Gx1[114]*Gu1[40] + Gx1[115]*Gu1[44] + Gx1[116]*Gu1[48];
Gu2[33] = + Gx1[104]*Gu1[1] + Gx1[105]*Gu1[5] + Gx1[106]*Gu1[9] + Gx1[107]*Gu1[13] + Gx1[108]*Gu1[17] + Gx1[109]*Gu1[21] + Gx1[110]*Gu1[25] + Gx1[111]*Gu1[29] + Gx1[112]*Gu1[33] + Gx1[113]*Gu1[37] + Gx1[114]*Gu1[41] + Gx1[115]*Gu1[45] + Gx1[116]*Gu1[49];
Gu2[34] = + Gx1[104]*Gu1[2] + Gx1[105]*Gu1[6] + Gx1[106]*Gu1[10] + Gx1[107]*Gu1[14] + Gx1[108]*Gu1[18] + Gx1[109]*Gu1[22] + Gx1[110]*Gu1[26] + Gx1[111]*Gu1[30] + Gx1[112]*Gu1[34] + Gx1[113]*Gu1[38] + Gx1[114]*Gu1[42] + Gx1[115]*Gu1[46] + Gx1[116]*Gu1[50];
Gu2[35] = + Gx1[104]*Gu1[3] + Gx1[105]*Gu1[7] + Gx1[106]*Gu1[11] + Gx1[107]*Gu1[15] + Gx1[108]*Gu1[19] + Gx1[109]*Gu1[23] + Gx1[110]*Gu1[27] + Gx1[111]*Gu1[31] + Gx1[112]*Gu1[35] + Gx1[113]*Gu1[39] + Gx1[114]*Gu1[43] + Gx1[115]*Gu1[47] + Gx1[116]*Gu1[51];
Gu2[36] = + Gx1[117]*Gu1[0] + Gx1[118]*Gu1[4] + Gx1[119]*Gu1[8] + Gx1[120]*Gu1[12] + Gx1[121]*Gu1[16] + Gx1[122]*Gu1[20] + Gx1[123]*Gu1[24] + Gx1[124]*Gu1[28] + Gx1[125]*Gu1[32] + Gx1[126]*Gu1[36] + Gx1[127]*Gu1[40] + Gx1[128]*Gu1[44] + Gx1[129]*Gu1[48];
Gu2[37] = + Gx1[117]*Gu1[1] + Gx1[118]*Gu1[5] + Gx1[119]*Gu1[9] + Gx1[120]*Gu1[13] + Gx1[121]*Gu1[17] + Gx1[122]*Gu1[21] + Gx1[123]*Gu1[25] + Gx1[124]*Gu1[29] + Gx1[125]*Gu1[33] + Gx1[126]*Gu1[37] + Gx1[127]*Gu1[41] + Gx1[128]*Gu1[45] + Gx1[129]*Gu1[49];
Gu2[38] = + Gx1[117]*Gu1[2] + Gx1[118]*Gu1[6] + Gx1[119]*Gu1[10] + Gx1[120]*Gu1[14] + Gx1[121]*Gu1[18] + Gx1[122]*Gu1[22] + Gx1[123]*Gu1[26] + Gx1[124]*Gu1[30] + Gx1[125]*Gu1[34] + Gx1[126]*Gu1[38] + Gx1[127]*Gu1[42] + Gx1[128]*Gu1[46] + Gx1[129]*Gu1[50];
Gu2[39] = + Gx1[117]*Gu1[3] + Gx1[118]*Gu1[7] + Gx1[119]*Gu1[11] + Gx1[120]*Gu1[15] + Gx1[121]*Gu1[19] + Gx1[122]*Gu1[23] + Gx1[123]*Gu1[27] + Gx1[124]*Gu1[31] + Gx1[125]*Gu1[35] + Gx1[126]*Gu1[39] + Gx1[127]*Gu1[43] + Gx1[128]*Gu1[47] + Gx1[129]*Gu1[51];
Gu2[40] = + Gx1[130]*Gu1[0] + Gx1[131]*Gu1[4] + Gx1[132]*Gu1[8] + Gx1[133]*Gu1[12] + Gx1[134]*Gu1[16] + Gx1[135]*Gu1[20] + Gx1[136]*Gu1[24] + Gx1[137]*Gu1[28] + Gx1[138]*Gu1[32] + Gx1[139]*Gu1[36] + Gx1[140]*Gu1[40] + Gx1[141]*Gu1[44] + Gx1[142]*Gu1[48];
Gu2[41] = + Gx1[130]*Gu1[1] + Gx1[131]*Gu1[5] + Gx1[132]*Gu1[9] + Gx1[133]*Gu1[13] + Gx1[134]*Gu1[17] + Gx1[135]*Gu1[21] + Gx1[136]*Gu1[25] + Gx1[137]*Gu1[29] + Gx1[138]*Gu1[33] + Gx1[139]*Gu1[37] + Gx1[140]*Gu1[41] + Gx1[141]*Gu1[45] + Gx1[142]*Gu1[49];
Gu2[42] = + Gx1[130]*Gu1[2] + Gx1[131]*Gu1[6] + Gx1[132]*Gu1[10] + Gx1[133]*Gu1[14] + Gx1[134]*Gu1[18] + Gx1[135]*Gu1[22] + Gx1[136]*Gu1[26] + Gx1[137]*Gu1[30] + Gx1[138]*Gu1[34] + Gx1[139]*Gu1[38] + Gx1[140]*Gu1[42] + Gx1[141]*Gu1[46] + Gx1[142]*Gu1[50];
Gu2[43] = + Gx1[130]*Gu1[3] + Gx1[131]*Gu1[7] + Gx1[132]*Gu1[11] + Gx1[133]*Gu1[15] + Gx1[134]*Gu1[19] + Gx1[135]*Gu1[23] + Gx1[136]*Gu1[27] + Gx1[137]*Gu1[31] + Gx1[138]*Gu1[35] + Gx1[139]*Gu1[39] + Gx1[140]*Gu1[43] + Gx1[141]*Gu1[47] + Gx1[142]*Gu1[51];
Gu2[44] = + Gx1[143]*Gu1[0] + Gx1[144]*Gu1[4] + Gx1[145]*Gu1[8] + Gx1[146]*Gu1[12] + Gx1[147]*Gu1[16] + Gx1[148]*Gu1[20] + Gx1[149]*Gu1[24] + Gx1[150]*Gu1[28] + Gx1[151]*Gu1[32] + Gx1[152]*Gu1[36] + Gx1[153]*Gu1[40] + Gx1[154]*Gu1[44] + Gx1[155]*Gu1[48];
Gu2[45] = + Gx1[143]*Gu1[1] + Gx1[144]*Gu1[5] + Gx1[145]*Gu1[9] + Gx1[146]*Gu1[13] + Gx1[147]*Gu1[17] + Gx1[148]*Gu1[21] + Gx1[149]*Gu1[25] + Gx1[150]*Gu1[29] + Gx1[151]*Gu1[33] + Gx1[152]*Gu1[37] + Gx1[153]*Gu1[41] + Gx1[154]*Gu1[45] + Gx1[155]*Gu1[49];
Gu2[46] = + Gx1[143]*Gu1[2] + Gx1[144]*Gu1[6] + Gx1[145]*Gu1[10] + Gx1[146]*Gu1[14] + Gx1[147]*Gu1[18] + Gx1[148]*Gu1[22] + Gx1[149]*Gu1[26] + Gx1[150]*Gu1[30] + Gx1[151]*Gu1[34] + Gx1[152]*Gu1[38] + Gx1[153]*Gu1[42] + Gx1[154]*Gu1[46] + Gx1[155]*Gu1[50];
Gu2[47] = + Gx1[143]*Gu1[3] + Gx1[144]*Gu1[7] + Gx1[145]*Gu1[11] + Gx1[146]*Gu1[15] + Gx1[147]*Gu1[19] + Gx1[148]*Gu1[23] + Gx1[149]*Gu1[27] + Gx1[150]*Gu1[31] + Gx1[151]*Gu1[35] + Gx1[152]*Gu1[39] + Gx1[153]*Gu1[43] + Gx1[154]*Gu1[47] + Gx1[155]*Gu1[51];
Gu2[48] = + Gx1[156]*Gu1[0] + Gx1[157]*Gu1[4] + Gx1[158]*Gu1[8] + Gx1[159]*Gu1[12] + Gx1[160]*Gu1[16] + Gx1[161]*Gu1[20] + Gx1[162]*Gu1[24] + Gx1[163]*Gu1[28] + Gx1[164]*Gu1[32] + Gx1[165]*Gu1[36] + Gx1[166]*Gu1[40] + Gx1[167]*Gu1[44] + Gx1[168]*Gu1[48];
Gu2[49] = + Gx1[156]*Gu1[1] + Gx1[157]*Gu1[5] + Gx1[158]*Gu1[9] + Gx1[159]*Gu1[13] + Gx1[160]*Gu1[17] + Gx1[161]*Gu1[21] + Gx1[162]*Gu1[25] + Gx1[163]*Gu1[29] + Gx1[164]*Gu1[33] + Gx1[165]*Gu1[37] + Gx1[166]*Gu1[41] + Gx1[167]*Gu1[45] + Gx1[168]*Gu1[49];
Gu2[50] = + Gx1[156]*Gu1[2] + Gx1[157]*Gu1[6] + Gx1[158]*Gu1[10] + Gx1[159]*Gu1[14] + Gx1[160]*Gu1[18] + Gx1[161]*Gu1[22] + Gx1[162]*Gu1[26] + Gx1[163]*Gu1[30] + Gx1[164]*Gu1[34] + Gx1[165]*Gu1[38] + Gx1[166]*Gu1[42] + Gx1[167]*Gu1[46] + Gx1[168]*Gu1[50];
Gu2[51] = + Gx1[156]*Gu1[3] + Gx1[157]*Gu1[7] + Gx1[158]*Gu1[11] + Gx1[159]*Gu1[15] + Gx1[160]*Gu1[19] + Gx1[161]*Gu1[23] + Gx1[162]*Gu1[27] + Gx1[163]*Gu1[31] + Gx1[164]*Gu1[35] + Gx1[165]*Gu1[39] + Gx1[166]*Gu1[43] + Gx1[167]*Gu1[47] + Gx1[168]*Gu1[51];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
Gu2[48] = Gu1[48];
Gu2[49] = Gu1[49];
Gu2[50] = Gu1[50];
Gu2[51] = Gu1[51];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + Gu1[48]*Gu2[48];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45] + Gu1[48]*Gu2[49];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46] + Gu1[48]*Gu2[50];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47] + Gu1[48]*Gu2[51];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44] + Gu1[49]*Gu2[48];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + Gu1[49]*Gu2[49];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46] + Gu1[49]*Gu2[50];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47] + Gu1[49]*Gu2[51];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44] + Gu1[50]*Gu2[48];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45] + Gu1[50]*Gu2[49];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + Gu1[50]*Gu2[50];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47] + Gu1[50]*Gu2[51];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44] + Gu1[51]*Gu2[48];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45] + Gu1[51]*Gu2[49];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46] + Gu1[51]*Gu2[50];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + Gu1[51]*Gu2[51];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 324] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + Gu1[48]*Gu2[48] + (real_t)6.0000000000000000e+00;
acadoWorkspace.H[iRow * 324 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45] + Gu1[48]*Gu2[49];
acadoWorkspace.H[iRow * 324 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46] + Gu1[48]*Gu2[50];
acadoWorkspace.H[iRow * 324 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47] + Gu1[48]*Gu2[51];
acadoWorkspace.H[iRow * 324 + 80] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44] + Gu1[49]*Gu2[48];
acadoWorkspace.H[iRow * 324 + 81] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + Gu1[49]*Gu2[49] + (real_t)6.0000000000000000e+00;
acadoWorkspace.H[iRow * 324 + 82] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46] + Gu1[49]*Gu2[50];
acadoWorkspace.H[iRow * 324 + 83] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47] + Gu1[49]*Gu2[51];
acadoWorkspace.H[iRow * 324 + 160] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44] + Gu1[50]*Gu2[48];
acadoWorkspace.H[iRow * 324 + 161] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45] + Gu1[50]*Gu2[49];
acadoWorkspace.H[iRow * 324 + 162] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + Gu1[50]*Gu2[50] + (real_t)6.0000000000000000e+00;
acadoWorkspace.H[iRow * 324 + 163] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47] + Gu1[50]*Gu2[51];
acadoWorkspace.H[iRow * 324 + 240] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44] + Gu1[51]*Gu2[48];
acadoWorkspace.H[iRow * 324 + 241] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45] + Gu1[51]*Gu2[49];
acadoWorkspace.H[iRow * 324 + 242] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46] + Gu1[51]*Gu2[50];
acadoWorkspace.H[iRow * 324 + 243] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + Gu1[51]*Gu2[51] + (real_t)6.0000000000000000e+00;
acadoWorkspace.H[iRow * 324] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 81] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 162] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 324 + 243] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[143]*Gu1[44] + Gx1[156]*Gu1[48];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[143]*Gu1[45] + Gx1[156]*Gu1[49];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[143]*Gu1[46] + Gx1[156]*Gu1[50];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[143]*Gu1[47] + Gx1[156]*Gu1[51];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[40]*Gu1[12] + Gx1[53]*Gu1[16] + Gx1[66]*Gu1[20] + Gx1[79]*Gu1[24] + Gx1[92]*Gu1[28] + Gx1[105]*Gu1[32] + Gx1[118]*Gu1[36] + Gx1[131]*Gu1[40] + Gx1[144]*Gu1[44] + Gx1[157]*Gu1[48];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[40]*Gu1[13] + Gx1[53]*Gu1[17] + Gx1[66]*Gu1[21] + Gx1[79]*Gu1[25] + Gx1[92]*Gu1[29] + Gx1[105]*Gu1[33] + Gx1[118]*Gu1[37] + Gx1[131]*Gu1[41] + Gx1[144]*Gu1[45] + Gx1[157]*Gu1[49];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[40]*Gu1[14] + Gx1[53]*Gu1[18] + Gx1[66]*Gu1[22] + Gx1[79]*Gu1[26] + Gx1[92]*Gu1[30] + Gx1[105]*Gu1[34] + Gx1[118]*Gu1[38] + Gx1[131]*Gu1[42] + Gx1[144]*Gu1[46] + Gx1[157]*Gu1[50];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[40]*Gu1[15] + Gx1[53]*Gu1[19] + Gx1[66]*Gu1[23] + Gx1[79]*Gu1[27] + Gx1[92]*Gu1[31] + Gx1[105]*Gu1[35] + Gx1[118]*Gu1[39] + Gx1[131]*Gu1[43] + Gx1[144]*Gu1[47] + Gx1[157]*Gu1[51];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[41]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[67]*Gu1[20] + Gx1[80]*Gu1[24] + Gx1[93]*Gu1[28] + Gx1[106]*Gu1[32] + Gx1[119]*Gu1[36] + Gx1[132]*Gu1[40] + Gx1[145]*Gu1[44] + Gx1[158]*Gu1[48];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[41]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[67]*Gu1[21] + Gx1[80]*Gu1[25] + Gx1[93]*Gu1[29] + Gx1[106]*Gu1[33] + Gx1[119]*Gu1[37] + Gx1[132]*Gu1[41] + Gx1[145]*Gu1[45] + Gx1[158]*Gu1[49];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[41]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[67]*Gu1[22] + Gx1[80]*Gu1[26] + Gx1[93]*Gu1[30] + Gx1[106]*Gu1[34] + Gx1[119]*Gu1[38] + Gx1[132]*Gu1[42] + Gx1[145]*Gu1[46] + Gx1[158]*Gu1[50];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[41]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[67]*Gu1[23] + Gx1[80]*Gu1[27] + Gx1[93]*Gu1[31] + Gx1[106]*Gu1[35] + Gx1[119]*Gu1[39] + Gx1[132]*Gu1[43] + Gx1[145]*Gu1[47] + Gx1[158]*Gu1[51];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[42]*Gu1[12] + Gx1[55]*Gu1[16] + Gx1[68]*Gu1[20] + Gx1[81]*Gu1[24] + Gx1[94]*Gu1[28] + Gx1[107]*Gu1[32] + Gx1[120]*Gu1[36] + Gx1[133]*Gu1[40] + Gx1[146]*Gu1[44] + Gx1[159]*Gu1[48];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[42]*Gu1[13] + Gx1[55]*Gu1[17] + Gx1[68]*Gu1[21] + Gx1[81]*Gu1[25] + Gx1[94]*Gu1[29] + Gx1[107]*Gu1[33] + Gx1[120]*Gu1[37] + Gx1[133]*Gu1[41] + Gx1[146]*Gu1[45] + Gx1[159]*Gu1[49];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[42]*Gu1[14] + Gx1[55]*Gu1[18] + Gx1[68]*Gu1[22] + Gx1[81]*Gu1[26] + Gx1[94]*Gu1[30] + Gx1[107]*Gu1[34] + Gx1[120]*Gu1[38] + Gx1[133]*Gu1[42] + Gx1[146]*Gu1[46] + Gx1[159]*Gu1[50];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[42]*Gu1[15] + Gx1[55]*Gu1[19] + Gx1[68]*Gu1[23] + Gx1[81]*Gu1[27] + Gx1[94]*Gu1[31] + Gx1[107]*Gu1[35] + Gx1[120]*Gu1[39] + Gx1[133]*Gu1[43] + Gx1[146]*Gu1[47] + Gx1[159]*Gu1[51];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[30]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[56]*Gu1[16] + Gx1[69]*Gu1[20] + Gx1[82]*Gu1[24] + Gx1[95]*Gu1[28] + Gx1[108]*Gu1[32] + Gx1[121]*Gu1[36] + Gx1[134]*Gu1[40] + Gx1[147]*Gu1[44] + Gx1[160]*Gu1[48];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[30]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[56]*Gu1[17] + Gx1[69]*Gu1[21] + Gx1[82]*Gu1[25] + Gx1[95]*Gu1[29] + Gx1[108]*Gu1[33] + Gx1[121]*Gu1[37] + Gx1[134]*Gu1[41] + Gx1[147]*Gu1[45] + Gx1[160]*Gu1[49];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[30]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[56]*Gu1[18] + Gx1[69]*Gu1[22] + Gx1[82]*Gu1[26] + Gx1[95]*Gu1[30] + Gx1[108]*Gu1[34] + Gx1[121]*Gu1[38] + Gx1[134]*Gu1[42] + Gx1[147]*Gu1[46] + Gx1[160]*Gu1[50];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[30]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[56]*Gu1[19] + Gx1[69]*Gu1[23] + Gx1[82]*Gu1[27] + Gx1[95]*Gu1[31] + Gx1[108]*Gu1[35] + Gx1[121]*Gu1[39] + Gx1[134]*Gu1[43] + Gx1[147]*Gu1[47] + Gx1[160]*Gu1[51];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[31]*Gu1[8] + Gx1[44]*Gu1[12] + Gx1[57]*Gu1[16] + Gx1[70]*Gu1[20] + Gx1[83]*Gu1[24] + Gx1[96]*Gu1[28] + Gx1[109]*Gu1[32] + Gx1[122]*Gu1[36] + Gx1[135]*Gu1[40] + Gx1[148]*Gu1[44] + Gx1[161]*Gu1[48];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[31]*Gu1[9] + Gx1[44]*Gu1[13] + Gx1[57]*Gu1[17] + Gx1[70]*Gu1[21] + Gx1[83]*Gu1[25] + Gx1[96]*Gu1[29] + Gx1[109]*Gu1[33] + Gx1[122]*Gu1[37] + Gx1[135]*Gu1[41] + Gx1[148]*Gu1[45] + Gx1[161]*Gu1[49];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[31]*Gu1[10] + Gx1[44]*Gu1[14] + Gx1[57]*Gu1[18] + Gx1[70]*Gu1[22] + Gx1[83]*Gu1[26] + Gx1[96]*Gu1[30] + Gx1[109]*Gu1[34] + Gx1[122]*Gu1[38] + Gx1[135]*Gu1[42] + Gx1[148]*Gu1[46] + Gx1[161]*Gu1[50];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[31]*Gu1[11] + Gx1[44]*Gu1[15] + Gx1[57]*Gu1[19] + Gx1[70]*Gu1[23] + Gx1[83]*Gu1[27] + Gx1[96]*Gu1[31] + Gx1[109]*Gu1[35] + Gx1[122]*Gu1[39] + Gx1[135]*Gu1[43] + Gx1[148]*Gu1[47] + Gx1[161]*Gu1[51];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[45]*Gu1[12] + Gx1[58]*Gu1[16] + Gx1[71]*Gu1[20] + Gx1[84]*Gu1[24] + Gx1[97]*Gu1[28] + Gx1[110]*Gu1[32] + Gx1[123]*Gu1[36] + Gx1[136]*Gu1[40] + Gx1[149]*Gu1[44] + Gx1[162]*Gu1[48];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[45]*Gu1[13] + Gx1[58]*Gu1[17] + Gx1[71]*Gu1[21] + Gx1[84]*Gu1[25] + Gx1[97]*Gu1[29] + Gx1[110]*Gu1[33] + Gx1[123]*Gu1[37] + Gx1[136]*Gu1[41] + Gx1[149]*Gu1[45] + Gx1[162]*Gu1[49];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[45]*Gu1[14] + Gx1[58]*Gu1[18] + Gx1[71]*Gu1[22] + Gx1[84]*Gu1[26] + Gx1[97]*Gu1[30] + Gx1[110]*Gu1[34] + Gx1[123]*Gu1[38] + Gx1[136]*Gu1[42] + Gx1[149]*Gu1[46] + Gx1[162]*Gu1[50];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[45]*Gu1[15] + Gx1[58]*Gu1[19] + Gx1[71]*Gu1[23] + Gx1[84]*Gu1[27] + Gx1[97]*Gu1[31] + Gx1[110]*Gu1[35] + Gx1[123]*Gu1[39] + Gx1[136]*Gu1[43] + Gx1[149]*Gu1[47] + Gx1[162]*Gu1[51];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[20]*Gu1[4] + Gx1[33]*Gu1[8] + Gx1[46]*Gu1[12] + Gx1[59]*Gu1[16] + Gx1[72]*Gu1[20] + Gx1[85]*Gu1[24] + Gx1[98]*Gu1[28] + Gx1[111]*Gu1[32] + Gx1[124]*Gu1[36] + Gx1[137]*Gu1[40] + Gx1[150]*Gu1[44] + Gx1[163]*Gu1[48];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[20]*Gu1[5] + Gx1[33]*Gu1[9] + Gx1[46]*Gu1[13] + Gx1[59]*Gu1[17] + Gx1[72]*Gu1[21] + Gx1[85]*Gu1[25] + Gx1[98]*Gu1[29] + Gx1[111]*Gu1[33] + Gx1[124]*Gu1[37] + Gx1[137]*Gu1[41] + Gx1[150]*Gu1[45] + Gx1[163]*Gu1[49];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[20]*Gu1[6] + Gx1[33]*Gu1[10] + Gx1[46]*Gu1[14] + Gx1[59]*Gu1[18] + Gx1[72]*Gu1[22] + Gx1[85]*Gu1[26] + Gx1[98]*Gu1[30] + Gx1[111]*Gu1[34] + Gx1[124]*Gu1[38] + Gx1[137]*Gu1[42] + Gx1[150]*Gu1[46] + Gx1[163]*Gu1[50];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[20]*Gu1[7] + Gx1[33]*Gu1[11] + Gx1[46]*Gu1[15] + Gx1[59]*Gu1[19] + Gx1[72]*Gu1[23] + Gx1[85]*Gu1[27] + Gx1[98]*Gu1[31] + Gx1[111]*Gu1[35] + Gx1[124]*Gu1[39] + Gx1[137]*Gu1[43] + Gx1[150]*Gu1[47] + Gx1[163]*Gu1[51];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[47]*Gu1[12] + Gx1[60]*Gu1[16] + Gx1[73]*Gu1[20] + Gx1[86]*Gu1[24] + Gx1[99]*Gu1[28] + Gx1[112]*Gu1[32] + Gx1[125]*Gu1[36] + Gx1[138]*Gu1[40] + Gx1[151]*Gu1[44] + Gx1[164]*Gu1[48];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[47]*Gu1[13] + Gx1[60]*Gu1[17] + Gx1[73]*Gu1[21] + Gx1[86]*Gu1[25] + Gx1[99]*Gu1[29] + Gx1[112]*Gu1[33] + Gx1[125]*Gu1[37] + Gx1[138]*Gu1[41] + Gx1[151]*Gu1[45] + Gx1[164]*Gu1[49];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[47]*Gu1[14] + Gx1[60]*Gu1[18] + Gx1[73]*Gu1[22] + Gx1[86]*Gu1[26] + Gx1[99]*Gu1[30] + Gx1[112]*Gu1[34] + Gx1[125]*Gu1[38] + Gx1[138]*Gu1[42] + Gx1[151]*Gu1[46] + Gx1[164]*Gu1[50];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[47]*Gu1[15] + Gx1[60]*Gu1[19] + Gx1[73]*Gu1[23] + Gx1[86]*Gu1[27] + Gx1[99]*Gu1[31] + Gx1[112]*Gu1[35] + Gx1[125]*Gu1[39] + Gx1[138]*Gu1[43] + Gx1[151]*Gu1[47] + Gx1[164]*Gu1[51];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[22]*Gu1[4] + Gx1[35]*Gu1[8] + Gx1[48]*Gu1[12] + Gx1[61]*Gu1[16] + Gx1[74]*Gu1[20] + Gx1[87]*Gu1[24] + Gx1[100]*Gu1[28] + Gx1[113]*Gu1[32] + Gx1[126]*Gu1[36] + Gx1[139]*Gu1[40] + Gx1[152]*Gu1[44] + Gx1[165]*Gu1[48];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[22]*Gu1[5] + Gx1[35]*Gu1[9] + Gx1[48]*Gu1[13] + Gx1[61]*Gu1[17] + Gx1[74]*Gu1[21] + Gx1[87]*Gu1[25] + Gx1[100]*Gu1[29] + Gx1[113]*Gu1[33] + Gx1[126]*Gu1[37] + Gx1[139]*Gu1[41] + Gx1[152]*Gu1[45] + Gx1[165]*Gu1[49];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[22]*Gu1[6] + Gx1[35]*Gu1[10] + Gx1[48]*Gu1[14] + Gx1[61]*Gu1[18] + Gx1[74]*Gu1[22] + Gx1[87]*Gu1[26] + Gx1[100]*Gu1[30] + Gx1[113]*Gu1[34] + Gx1[126]*Gu1[38] + Gx1[139]*Gu1[42] + Gx1[152]*Gu1[46] + Gx1[165]*Gu1[50];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[22]*Gu1[7] + Gx1[35]*Gu1[11] + Gx1[48]*Gu1[15] + Gx1[61]*Gu1[19] + Gx1[74]*Gu1[23] + Gx1[87]*Gu1[27] + Gx1[100]*Gu1[31] + Gx1[113]*Gu1[35] + Gx1[126]*Gu1[39] + Gx1[139]*Gu1[43] + Gx1[152]*Gu1[47] + Gx1[165]*Gu1[51];
Gu2[40] = + Gx1[10]*Gu1[0] + Gx1[23]*Gu1[4] + Gx1[36]*Gu1[8] + Gx1[49]*Gu1[12] + Gx1[62]*Gu1[16] + Gx1[75]*Gu1[20] + Gx1[88]*Gu1[24] + Gx1[101]*Gu1[28] + Gx1[114]*Gu1[32] + Gx1[127]*Gu1[36] + Gx1[140]*Gu1[40] + Gx1[153]*Gu1[44] + Gx1[166]*Gu1[48];
Gu2[41] = + Gx1[10]*Gu1[1] + Gx1[23]*Gu1[5] + Gx1[36]*Gu1[9] + Gx1[49]*Gu1[13] + Gx1[62]*Gu1[17] + Gx1[75]*Gu1[21] + Gx1[88]*Gu1[25] + Gx1[101]*Gu1[29] + Gx1[114]*Gu1[33] + Gx1[127]*Gu1[37] + Gx1[140]*Gu1[41] + Gx1[153]*Gu1[45] + Gx1[166]*Gu1[49];
Gu2[42] = + Gx1[10]*Gu1[2] + Gx1[23]*Gu1[6] + Gx1[36]*Gu1[10] + Gx1[49]*Gu1[14] + Gx1[62]*Gu1[18] + Gx1[75]*Gu1[22] + Gx1[88]*Gu1[26] + Gx1[101]*Gu1[30] + Gx1[114]*Gu1[34] + Gx1[127]*Gu1[38] + Gx1[140]*Gu1[42] + Gx1[153]*Gu1[46] + Gx1[166]*Gu1[50];
Gu2[43] = + Gx1[10]*Gu1[3] + Gx1[23]*Gu1[7] + Gx1[36]*Gu1[11] + Gx1[49]*Gu1[15] + Gx1[62]*Gu1[19] + Gx1[75]*Gu1[23] + Gx1[88]*Gu1[27] + Gx1[101]*Gu1[31] + Gx1[114]*Gu1[35] + Gx1[127]*Gu1[39] + Gx1[140]*Gu1[43] + Gx1[153]*Gu1[47] + Gx1[166]*Gu1[51];
Gu2[44] = + Gx1[11]*Gu1[0] + Gx1[24]*Gu1[4] + Gx1[37]*Gu1[8] + Gx1[50]*Gu1[12] + Gx1[63]*Gu1[16] + Gx1[76]*Gu1[20] + Gx1[89]*Gu1[24] + Gx1[102]*Gu1[28] + Gx1[115]*Gu1[32] + Gx1[128]*Gu1[36] + Gx1[141]*Gu1[40] + Gx1[154]*Gu1[44] + Gx1[167]*Gu1[48];
Gu2[45] = + Gx1[11]*Gu1[1] + Gx1[24]*Gu1[5] + Gx1[37]*Gu1[9] + Gx1[50]*Gu1[13] + Gx1[63]*Gu1[17] + Gx1[76]*Gu1[21] + Gx1[89]*Gu1[25] + Gx1[102]*Gu1[29] + Gx1[115]*Gu1[33] + Gx1[128]*Gu1[37] + Gx1[141]*Gu1[41] + Gx1[154]*Gu1[45] + Gx1[167]*Gu1[49];
Gu2[46] = + Gx1[11]*Gu1[2] + Gx1[24]*Gu1[6] + Gx1[37]*Gu1[10] + Gx1[50]*Gu1[14] + Gx1[63]*Gu1[18] + Gx1[76]*Gu1[22] + Gx1[89]*Gu1[26] + Gx1[102]*Gu1[30] + Gx1[115]*Gu1[34] + Gx1[128]*Gu1[38] + Gx1[141]*Gu1[42] + Gx1[154]*Gu1[46] + Gx1[167]*Gu1[50];
Gu2[47] = + Gx1[11]*Gu1[3] + Gx1[24]*Gu1[7] + Gx1[37]*Gu1[11] + Gx1[50]*Gu1[15] + Gx1[63]*Gu1[19] + Gx1[76]*Gu1[23] + Gx1[89]*Gu1[27] + Gx1[102]*Gu1[31] + Gx1[115]*Gu1[35] + Gx1[128]*Gu1[39] + Gx1[141]*Gu1[43] + Gx1[154]*Gu1[47] + Gx1[167]*Gu1[51];
Gu2[48] = + Gx1[12]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[155]*Gu1[44] + Gx1[168]*Gu1[48];
Gu2[49] = + Gx1[12]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[155]*Gu1[45] + Gx1[168]*Gu1[49];
Gu2[50] = + Gx1[12]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[155]*Gu1[46] + Gx1[168]*Gu1[50];
Gu2[51] = + Gx1[12]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[155]*Gu1[47] + Gx1[168]*Gu1[51];
}

void acado_multQEW2( real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + (real_t)2.0000000000000000e+02*Gu1[0] + Gu2[0];
Gu3[1] = + (real_t)2.0000000000000000e+02*Gu1[1] + Gu2[1];
Gu3[2] = + (real_t)2.0000000000000000e+02*Gu1[2] + Gu2[2];
Gu3[3] = + (real_t)2.0000000000000000e+02*Gu1[3] + Gu2[3];
Gu3[4] = + (real_t)2.0000000000000000e+02*Gu1[4] + Gu2[4];
Gu3[5] = + (real_t)2.0000000000000000e+02*Gu1[5] + Gu2[5];
Gu3[6] = + (real_t)2.0000000000000000e+02*Gu1[6] + Gu2[6];
Gu3[7] = + (real_t)2.0000000000000000e+02*Gu1[7] + Gu2[7];
Gu3[8] = + (real_t)3.0000000000000000e+02*Gu1[8] + Gu2[8];
Gu3[9] = + (real_t)3.0000000000000000e+02*Gu1[9] + Gu2[9];
Gu3[10] = + (real_t)3.0000000000000000e+02*Gu1[10] + Gu2[10];
Gu3[11] = + (real_t)3.0000000000000000e+02*Gu1[11] + Gu2[11];
Gu3[12] = +Gu1[12] + Gu2[12];
Gu3[13] = +Gu1[13] + Gu2[13];
Gu3[14] = +Gu1[14] + Gu2[14];
Gu3[15] = +Gu1[15] + Gu2[15];
Gu3[16] = +Gu1[16] + Gu2[16];
Gu3[17] = +Gu1[17] + Gu2[17];
Gu3[18] = +Gu1[18] + Gu2[18];
Gu3[19] = +Gu1[19] + Gu2[19];
Gu3[20] = +Gu1[20] + Gu2[20];
Gu3[21] = +Gu1[21] + Gu2[21];
Gu3[22] = +Gu1[22] + Gu2[22];
Gu3[23] = +Gu1[23] + Gu2[23];
Gu3[24] = + Gu2[24];
Gu3[25] = + Gu2[25];
Gu3[26] = + Gu2[26];
Gu3[27] = + Gu2[27];
Gu3[28] = + (real_t)5.0000000000000000e+00*Gu1[28] + Gu2[28];
Gu3[29] = + (real_t)5.0000000000000000e+00*Gu1[29] + Gu2[29];
Gu3[30] = + (real_t)5.0000000000000000e+00*Gu1[30] + Gu2[30];
Gu3[31] = + (real_t)5.0000000000000000e+00*Gu1[31] + Gu2[31];
Gu3[32] = + (real_t)5.0000000000000000e+00*Gu1[32] + Gu2[32];
Gu3[33] = + (real_t)5.0000000000000000e+00*Gu1[33] + Gu2[33];
Gu3[34] = + (real_t)5.0000000000000000e+00*Gu1[34] + Gu2[34];
Gu3[35] = + (real_t)5.0000000000000000e+00*Gu1[35] + Gu2[35];
Gu3[36] = + (real_t)2.0000000000000000e+02*Gu1[36] + Gu2[36];
Gu3[37] = + (real_t)2.0000000000000000e+02*Gu1[37] + Gu2[37];
Gu3[38] = + (real_t)2.0000000000000000e+02*Gu1[38] + Gu2[38];
Gu3[39] = + (real_t)2.0000000000000000e+02*Gu1[39] + Gu2[39];
Gu3[40] = +Gu1[40] + Gu2[40];
Gu3[41] = +Gu1[41] + Gu2[41];
Gu3[42] = +Gu1[42] + Gu2[42];
Gu3[43] = +Gu1[43] + Gu2[43];
Gu3[44] = +Gu1[44] + Gu2[44];
Gu3[45] = +Gu1[45] + Gu2[45];
Gu3[46] = +Gu1[46] + Gu2[46];
Gu3[47] = +Gu1[47] + Gu2[47];
Gu3[48] = +Gu1[48] + Gu2[48];
Gu3[49] = +Gu1[49] + Gu2[49];
Gu3[50] = +Gu1[50] + Gu2[50];
Gu3[51] = +Gu1[51] + Gu2[51];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[13]*w11[1] + Gx1[26]*w11[2] + Gx1[39]*w11[3] + Gx1[52]*w11[4] + Gx1[65]*w11[5] + Gx1[78]*w11[6] + Gx1[91]*w11[7] + Gx1[104]*w11[8] + Gx1[117]*w11[9] + Gx1[130]*w11[10] + Gx1[143]*w11[11] + Gx1[156]*w11[12] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[14]*w11[1] + Gx1[27]*w11[2] + Gx1[40]*w11[3] + Gx1[53]*w11[4] + Gx1[66]*w11[5] + Gx1[79]*w11[6] + Gx1[92]*w11[7] + Gx1[105]*w11[8] + Gx1[118]*w11[9] + Gx1[131]*w11[10] + Gx1[144]*w11[11] + Gx1[157]*w11[12] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[15]*w11[1] + Gx1[28]*w11[2] + Gx1[41]*w11[3] + Gx1[54]*w11[4] + Gx1[67]*w11[5] + Gx1[80]*w11[6] + Gx1[93]*w11[7] + Gx1[106]*w11[8] + Gx1[119]*w11[9] + Gx1[132]*w11[10] + Gx1[145]*w11[11] + Gx1[158]*w11[12] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[16]*w11[1] + Gx1[29]*w11[2] + Gx1[42]*w11[3] + Gx1[55]*w11[4] + Gx1[68]*w11[5] + Gx1[81]*w11[6] + Gx1[94]*w11[7] + Gx1[107]*w11[8] + Gx1[120]*w11[9] + Gx1[133]*w11[10] + Gx1[146]*w11[11] + Gx1[159]*w11[12] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[17]*w11[1] + Gx1[30]*w11[2] + Gx1[43]*w11[3] + Gx1[56]*w11[4] + Gx1[69]*w11[5] + Gx1[82]*w11[6] + Gx1[95]*w11[7] + Gx1[108]*w11[8] + Gx1[121]*w11[9] + Gx1[134]*w11[10] + Gx1[147]*w11[11] + Gx1[160]*w11[12] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[18]*w11[1] + Gx1[31]*w11[2] + Gx1[44]*w11[3] + Gx1[57]*w11[4] + Gx1[70]*w11[5] + Gx1[83]*w11[6] + Gx1[96]*w11[7] + Gx1[109]*w11[8] + Gx1[122]*w11[9] + Gx1[135]*w11[10] + Gx1[148]*w11[11] + Gx1[161]*w11[12] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[19]*w11[1] + Gx1[32]*w11[2] + Gx1[45]*w11[3] + Gx1[58]*w11[4] + Gx1[71]*w11[5] + Gx1[84]*w11[6] + Gx1[97]*w11[7] + Gx1[110]*w11[8] + Gx1[123]*w11[9] + Gx1[136]*w11[10] + Gx1[149]*w11[11] + Gx1[162]*w11[12] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[20]*w11[1] + Gx1[33]*w11[2] + Gx1[46]*w11[3] + Gx1[59]*w11[4] + Gx1[72]*w11[5] + Gx1[85]*w11[6] + Gx1[98]*w11[7] + Gx1[111]*w11[8] + Gx1[124]*w11[9] + Gx1[137]*w11[10] + Gx1[150]*w11[11] + Gx1[163]*w11[12] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[21]*w11[1] + Gx1[34]*w11[2] + Gx1[47]*w11[3] + Gx1[60]*w11[4] + Gx1[73]*w11[5] + Gx1[86]*w11[6] + Gx1[99]*w11[7] + Gx1[112]*w11[8] + Gx1[125]*w11[9] + Gx1[138]*w11[10] + Gx1[151]*w11[11] + Gx1[164]*w11[12] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[22]*w11[1] + Gx1[35]*w11[2] + Gx1[48]*w11[3] + Gx1[61]*w11[4] + Gx1[74]*w11[5] + Gx1[87]*w11[6] + Gx1[100]*w11[7] + Gx1[113]*w11[8] + Gx1[126]*w11[9] + Gx1[139]*w11[10] + Gx1[152]*w11[11] + Gx1[165]*w11[12] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[23]*w11[1] + Gx1[36]*w11[2] + Gx1[49]*w11[3] + Gx1[62]*w11[4] + Gx1[75]*w11[5] + Gx1[88]*w11[6] + Gx1[101]*w11[7] + Gx1[114]*w11[8] + Gx1[127]*w11[9] + Gx1[140]*w11[10] + Gx1[153]*w11[11] + Gx1[166]*w11[12] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[24]*w11[1] + Gx1[37]*w11[2] + Gx1[50]*w11[3] + Gx1[63]*w11[4] + Gx1[76]*w11[5] + Gx1[89]*w11[6] + Gx1[102]*w11[7] + Gx1[115]*w11[8] + Gx1[128]*w11[9] + Gx1[141]*w11[10] + Gx1[154]*w11[11] + Gx1[167]*w11[12] + w12[11];
w13[12] = + Gx1[12]*w11[0] + Gx1[25]*w11[1] + Gx1[38]*w11[2] + Gx1[51]*w11[3] + Gx1[64]*w11[4] + Gx1[77]*w11[5] + Gx1[90]*w11[6] + Gx1[103]*w11[7] + Gx1[116]*w11[8] + Gx1[129]*w11[9] + Gx1[142]*w11[10] + Gx1[155]*w11[11] + Gx1[168]*w11[12] + w12[12];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9] + Gu1[40]*w11[10] + Gu1[44]*w11[11] + Gu1[48]*w11[12];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9] + Gu1[41]*w11[10] + Gu1[45]*w11[11] + Gu1[49]*w11[12];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9] + Gu1[42]*w11[10] + Gu1[46]*w11[11] + Gu1[50]*w11[12];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9] + Gu1[43]*w11[10] + Gu1[47]*w11[11] + Gu1[51]*w11[12];
}

void acado_macQSbarW2( real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + (real_t)2.0000000000000000e+02*w11[0] + w12[0];
w13[1] = + (real_t)2.0000000000000000e+02*w11[1] + w12[1];
w13[2] = + (real_t)3.0000000000000000e+02*w11[2] + w12[2];
w13[3] = +w11[3] + w12[3];
w13[4] = +w11[4] + w12[4];
w13[5] = +w11[5] + w12[5];
w13[6] = + w12[6];
w13[7] = + (real_t)5.0000000000000000e+00*w11[7] + w12[7];
w13[8] = + (real_t)5.0000000000000000e+00*w11[8] + w12[8];
w13[9] = + (real_t)2.0000000000000000e+02*w11[9] + w12[9];
w13[10] = +w11[10] + w12[10];
w13[11] = +w11[11] + w12[11];
w13[12] = +w11[12] + w12[12];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12];
w12[1] += + Gx1[13]*w11[0] + Gx1[14]*w11[1] + Gx1[15]*w11[2] + Gx1[16]*w11[3] + Gx1[17]*w11[4] + Gx1[18]*w11[5] + Gx1[19]*w11[6] + Gx1[20]*w11[7] + Gx1[21]*w11[8] + Gx1[22]*w11[9] + Gx1[23]*w11[10] + Gx1[24]*w11[11] + Gx1[25]*w11[12];
w12[2] += + Gx1[26]*w11[0] + Gx1[27]*w11[1] + Gx1[28]*w11[2] + Gx1[29]*w11[3] + Gx1[30]*w11[4] + Gx1[31]*w11[5] + Gx1[32]*w11[6] + Gx1[33]*w11[7] + Gx1[34]*w11[8] + Gx1[35]*w11[9] + Gx1[36]*w11[10] + Gx1[37]*w11[11] + Gx1[38]*w11[12];
w12[3] += + Gx1[39]*w11[0] + Gx1[40]*w11[1] + Gx1[41]*w11[2] + Gx1[42]*w11[3] + Gx1[43]*w11[4] + Gx1[44]*w11[5] + Gx1[45]*w11[6] + Gx1[46]*w11[7] + Gx1[47]*w11[8] + Gx1[48]*w11[9] + Gx1[49]*w11[10] + Gx1[50]*w11[11] + Gx1[51]*w11[12];
w12[4] += + Gx1[52]*w11[0] + Gx1[53]*w11[1] + Gx1[54]*w11[2] + Gx1[55]*w11[3] + Gx1[56]*w11[4] + Gx1[57]*w11[5] + Gx1[58]*w11[6] + Gx1[59]*w11[7] + Gx1[60]*w11[8] + Gx1[61]*w11[9] + Gx1[62]*w11[10] + Gx1[63]*w11[11] + Gx1[64]*w11[12];
w12[5] += + Gx1[65]*w11[0] + Gx1[66]*w11[1] + Gx1[67]*w11[2] + Gx1[68]*w11[3] + Gx1[69]*w11[4] + Gx1[70]*w11[5] + Gx1[71]*w11[6] + Gx1[72]*w11[7] + Gx1[73]*w11[8] + Gx1[74]*w11[9] + Gx1[75]*w11[10] + Gx1[76]*w11[11] + Gx1[77]*w11[12];
w12[6] += + Gx1[78]*w11[0] + Gx1[79]*w11[1] + Gx1[80]*w11[2] + Gx1[81]*w11[3] + Gx1[82]*w11[4] + Gx1[83]*w11[5] + Gx1[84]*w11[6] + Gx1[85]*w11[7] + Gx1[86]*w11[8] + Gx1[87]*w11[9] + Gx1[88]*w11[10] + Gx1[89]*w11[11] + Gx1[90]*w11[12];
w12[7] += + Gx1[91]*w11[0] + Gx1[92]*w11[1] + Gx1[93]*w11[2] + Gx1[94]*w11[3] + Gx1[95]*w11[4] + Gx1[96]*w11[5] + Gx1[97]*w11[6] + Gx1[98]*w11[7] + Gx1[99]*w11[8] + Gx1[100]*w11[9] + Gx1[101]*w11[10] + Gx1[102]*w11[11] + Gx1[103]*w11[12];
w12[8] += + Gx1[104]*w11[0] + Gx1[105]*w11[1] + Gx1[106]*w11[2] + Gx1[107]*w11[3] + Gx1[108]*w11[4] + Gx1[109]*w11[5] + Gx1[110]*w11[6] + Gx1[111]*w11[7] + Gx1[112]*w11[8] + Gx1[113]*w11[9] + Gx1[114]*w11[10] + Gx1[115]*w11[11] + Gx1[116]*w11[12];
w12[9] += + Gx1[117]*w11[0] + Gx1[118]*w11[1] + Gx1[119]*w11[2] + Gx1[120]*w11[3] + Gx1[121]*w11[4] + Gx1[122]*w11[5] + Gx1[123]*w11[6] + Gx1[124]*w11[7] + Gx1[125]*w11[8] + Gx1[126]*w11[9] + Gx1[127]*w11[10] + Gx1[128]*w11[11] + Gx1[129]*w11[12];
w12[10] += + Gx1[130]*w11[0] + Gx1[131]*w11[1] + Gx1[132]*w11[2] + Gx1[133]*w11[3] + Gx1[134]*w11[4] + Gx1[135]*w11[5] + Gx1[136]*w11[6] + Gx1[137]*w11[7] + Gx1[138]*w11[8] + Gx1[139]*w11[9] + Gx1[140]*w11[10] + Gx1[141]*w11[11] + Gx1[142]*w11[12];
w12[11] += + Gx1[143]*w11[0] + Gx1[144]*w11[1] + Gx1[145]*w11[2] + Gx1[146]*w11[3] + Gx1[147]*w11[4] + Gx1[148]*w11[5] + Gx1[149]*w11[6] + Gx1[150]*w11[7] + Gx1[151]*w11[8] + Gx1[152]*w11[9] + Gx1[153]*w11[10] + Gx1[154]*w11[11] + Gx1[155]*w11[12];
w12[12] += + Gx1[156]*w11[0] + Gx1[157]*w11[1] + Gx1[158]*w11[2] + Gx1[159]*w11[3] + Gx1[160]*w11[4] + Gx1[161]*w11[5] + Gx1[162]*w11[6] + Gx1[163]*w11[7] + Gx1[164]*w11[8] + Gx1[165]*w11[9] + Gx1[166]*w11[10] + Gx1[167]*w11[11] + Gx1[168]*w11[12];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12];
w12[1] += + Gx1[13]*w11[0] + Gx1[14]*w11[1] + Gx1[15]*w11[2] + Gx1[16]*w11[3] + Gx1[17]*w11[4] + Gx1[18]*w11[5] + Gx1[19]*w11[6] + Gx1[20]*w11[7] + Gx1[21]*w11[8] + Gx1[22]*w11[9] + Gx1[23]*w11[10] + Gx1[24]*w11[11] + Gx1[25]*w11[12];
w12[2] += + Gx1[26]*w11[0] + Gx1[27]*w11[1] + Gx1[28]*w11[2] + Gx1[29]*w11[3] + Gx1[30]*w11[4] + Gx1[31]*w11[5] + Gx1[32]*w11[6] + Gx1[33]*w11[7] + Gx1[34]*w11[8] + Gx1[35]*w11[9] + Gx1[36]*w11[10] + Gx1[37]*w11[11] + Gx1[38]*w11[12];
w12[3] += + Gx1[39]*w11[0] + Gx1[40]*w11[1] + Gx1[41]*w11[2] + Gx1[42]*w11[3] + Gx1[43]*w11[4] + Gx1[44]*w11[5] + Gx1[45]*w11[6] + Gx1[46]*w11[7] + Gx1[47]*w11[8] + Gx1[48]*w11[9] + Gx1[49]*w11[10] + Gx1[50]*w11[11] + Gx1[51]*w11[12];
w12[4] += + Gx1[52]*w11[0] + Gx1[53]*w11[1] + Gx1[54]*w11[2] + Gx1[55]*w11[3] + Gx1[56]*w11[4] + Gx1[57]*w11[5] + Gx1[58]*w11[6] + Gx1[59]*w11[7] + Gx1[60]*w11[8] + Gx1[61]*w11[9] + Gx1[62]*w11[10] + Gx1[63]*w11[11] + Gx1[64]*w11[12];
w12[5] += + Gx1[65]*w11[0] + Gx1[66]*w11[1] + Gx1[67]*w11[2] + Gx1[68]*w11[3] + Gx1[69]*w11[4] + Gx1[70]*w11[5] + Gx1[71]*w11[6] + Gx1[72]*w11[7] + Gx1[73]*w11[8] + Gx1[74]*w11[9] + Gx1[75]*w11[10] + Gx1[76]*w11[11] + Gx1[77]*w11[12];
w12[6] += + Gx1[78]*w11[0] + Gx1[79]*w11[1] + Gx1[80]*w11[2] + Gx1[81]*w11[3] + Gx1[82]*w11[4] + Gx1[83]*w11[5] + Gx1[84]*w11[6] + Gx1[85]*w11[7] + Gx1[86]*w11[8] + Gx1[87]*w11[9] + Gx1[88]*w11[10] + Gx1[89]*w11[11] + Gx1[90]*w11[12];
w12[7] += + Gx1[91]*w11[0] + Gx1[92]*w11[1] + Gx1[93]*w11[2] + Gx1[94]*w11[3] + Gx1[95]*w11[4] + Gx1[96]*w11[5] + Gx1[97]*w11[6] + Gx1[98]*w11[7] + Gx1[99]*w11[8] + Gx1[100]*w11[9] + Gx1[101]*w11[10] + Gx1[102]*w11[11] + Gx1[103]*w11[12];
w12[8] += + Gx1[104]*w11[0] + Gx1[105]*w11[1] + Gx1[106]*w11[2] + Gx1[107]*w11[3] + Gx1[108]*w11[4] + Gx1[109]*w11[5] + Gx1[110]*w11[6] + Gx1[111]*w11[7] + Gx1[112]*w11[8] + Gx1[113]*w11[9] + Gx1[114]*w11[10] + Gx1[115]*w11[11] + Gx1[116]*w11[12];
w12[9] += + Gx1[117]*w11[0] + Gx1[118]*w11[1] + Gx1[119]*w11[2] + Gx1[120]*w11[3] + Gx1[121]*w11[4] + Gx1[122]*w11[5] + Gx1[123]*w11[6] + Gx1[124]*w11[7] + Gx1[125]*w11[8] + Gx1[126]*w11[9] + Gx1[127]*w11[10] + Gx1[128]*w11[11] + Gx1[129]*w11[12];
w12[10] += + Gx1[130]*w11[0] + Gx1[131]*w11[1] + Gx1[132]*w11[2] + Gx1[133]*w11[3] + Gx1[134]*w11[4] + Gx1[135]*w11[5] + Gx1[136]*w11[6] + Gx1[137]*w11[7] + Gx1[138]*w11[8] + Gx1[139]*w11[9] + Gx1[140]*w11[10] + Gx1[141]*w11[11] + Gx1[142]*w11[12];
w12[11] += + Gx1[143]*w11[0] + Gx1[144]*w11[1] + Gx1[145]*w11[2] + Gx1[146]*w11[3] + Gx1[147]*w11[4] + Gx1[148]*w11[5] + Gx1[149]*w11[6] + Gx1[150]*w11[7] + Gx1[151]*w11[8] + Gx1[152]*w11[9] + Gx1[153]*w11[10] + Gx1[154]*w11[11] + Gx1[155]*w11[12];
w12[12] += + Gx1[156]*w11[0] + Gx1[157]*w11[1] + Gx1[158]*w11[2] + Gx1[159]*w11[3] + Gx1[160]*w11[4] + Gx1[161]*w11[5] + Gx1[162]*w11[6] + Gx1[163]*w11[7] + Gx1[164]*w11[8] + Gx1[165]*w11[9] + Gx1[166]*w11[10] + Gx1[167]*w11[11] + Gx1[168]*w11[12];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
w12[10] += + Gu1[40]*U1[0] + Gu1[41]*U1[1] + Gu1[42]*U1[2] + Gu1[43]*U1[3];
w12[11] += + Gu1[44]*U1[0] + Gu1[45]*U1[1] + Gu1[46]*U1[2] + Gu1[47]*U1[3];
w12[12] += + Gu1[48]*U1[0] + Gu1[49]*U1[1] + Gu1[50]*U1[2] + Gu1[51]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)6.0000000000000000e+00*Dy1[12];
RDy1[1] = + (real_t)6.0000000000000000e+00*Dy1[13];
RDy1[2] = + (real_t)6.0000000000000000e+00*Dy1[14];
RDy1[3] = + (real_t)6.0000000000000000e+00*Dy1[15];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)2.0000000000000000e+02*Dy1[0];
QDy1[1] = + (real_t)2.0000000000000000e+02*Dy1[1];
QDy1[2] = + (real_t)3.0000000000000000e+02*Dy1[2];
QDy1[3] = +Dy1[3];
QDy1[4] = +Dy1[4];
QDy1[5] = +Dy1[5];
QDy1[6] = 0.0;
;
QDy1[7] = + (real_t)5.0000000000000000e+00*Dy1[6];
QDy1[8] = + (real_t)5.0000000000000000e+00*Dy1[7];
QDy1[9] = + (real_t)2.0000000000000000e+02*Dy1[8];
QDy1[10] = +Dy1[9];
QDy1[11] = +Dy1[10];
QDy1[12] = +Dy1[11];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)2.0000000000000000e+02*Gx1[0];
Gx2[1] = + (real_t)2.0000000000000000e+02*Gx1[1];
Gx2[2] = + (real_t)2.0000000000000000e+02*Gx1[2];
Gx2[3] = + (real_t)2.0000000000000000e+02*Gx1[3];
Gx2[4] = + (real_t)2.0000000000000000e+02*Gx1[4];
Gx2[5] = + (real_t)2.0000000000000000e+02*Gx1[5];
Gx2[6] = + (real_t)2.0000000000000000e+02*Gx1[6];
Gx2[7] = + (real_t)2.0000000000000000e+02*Gx1[7];
Gx2[8] = + (real_t)2.0000000000000000e+02*Gx1[8];
Gx2[9] = + (real_t)2.0000000000000000e+02*Gx1[9];
Gx2[10] = + (real_t)2.0000000000000000e+02*Gx1[10];
Gx2[11] = + (real_t)2.0000000000000000e+02*Gx1[11];
Gx2[12] = + (real_t)2.0000000000000000e+02*Gx1[12];
Gx2[13] = + (real_t)2.0000000000000000e+02*Gx1[13];
Gx2[14] = + (real_t)2.0000000000000000e+02*Gx1[14];
Gx2[15] = + (real_t)2.0000000000000000e+02*Gx1[15];
Gx2[16] = + (real_t)2.0000000000000000e+02*Gx1[16];
Gx2[17] = + (real_t)2.0000000000000000e+02*Gx1[17];
Gx2[18] = + (real_t)2.0000000000000000e+02*Gx1[18];
Gx2[19] = + (real_t)2.0000000000000000e+02*Gx1[19];
Gx2[20] = + (real_t)2.0000000000000000e+02*Gx1[20];
Gx2[21] = + (real_t)2.0000000000000000e+02*Gx1[21];
Gx2[22] = + (real_t)2.0000000000000000e+02*Gx1[22];
Gx2[23] = + (real_t)2.0000000000000000e+02*Gx1[23];
Gx2[24] = + (real_t)2.0000000000000000e+02*Gx1[24];
Gx2[25] = + (real_t)2.0000000000000000e+02*Gx1[25];
Gx2[26] = + (real_t)3.0000000000000000e+02*Gx1[26];
Gx2[27] = + (real_t)3.0000000000000000e+02*Gx1[27];
Gx2[28] = + (real_t)3.0000000000000000e+02*Gx1[28];
Gx2[29] = + (real_t)3.0000000000000000e+02*Gx1[29];
Gx2[30] = + (real_t)3.0000000000000000e+02*Gx1[30];
Gx2[31] = + (real_t)3.0000000000000000e+02*Gx1[31];
Gx2[32] = + (real_t)3.0000000000000000e+02*Gx1[32];
Gx2[33] = + (real_t)3.0000000000000000e+02*Gx1[33];
Gx2[34] = + (real_t)3.0000000000000000e+02*Gx1[34];
Gx2[35] = + (real_t)3.0000000000000000e+02*Gx1[35];
Gx2[36] = + (real_t)3.0000000000000000e+02*Gx1[36];
Gx2[37] = + (real_t)3.0000000000000000e+02*Gx1[37];
Gx2[38] = + (real_t)3.0000000000000000e+02*Gx1[38];
Gx2[39] = +Gx1[39];
Gx2[40] = +Gx1[40];
Gx2[41] = +Gx1[41];
Gx2[42] = +Gx1[42];
Gx2[43] = +Gx1[43];
Gx2[44] = +Gx1[44];
Gx2[45] = +Gx1[45];
Gx2[46] = +Gx1[46];
Gx2[47] = +Gx1[47];
Gx2[48] = +Gx1[48];
Gx2[49] = +Gx1[49];
Gx2[50] = +Gx1[50];
Gx2[51] = +Gx1[51];
Gx2[52] = +Gx1[52];
Gx2[53] = +Gx1[53];
Gx2[54] = +Gx1[54];
Gx2[55] = +Gx1[55];
Gx2[56] = +Gx1[56];
Gx2[57] = +Gx1[57];
Gx2[58] = +Gx1[58];
Gx2[59] = +Gx1[59];
Gx2[60] = +Gx1[60];
Gx2[61] = +Gx1[61];
Gx2[62] = +Gx1[62];
Gx2[63] = +Gx1[63];
Gx2[64] = +Gx1[64];
Gx2[65] = +Gx1[65];
Gx2[66] = +Gx1[66];
Gx2[67] = +Gx1[67];
Gx2[68] = +Gx1[68];
Gx2[69] = +Gx1[69];
Gx2[70] = +Gx1[70];
Gx2[71] = +Gx1[71];
Gx2[72] = +Gx1[72];
Gx2[73] = +Gx1[73];
Gx2[74] = +Gx1[74];
Gx2[75] = +Gx1[75];
Gx2[76] = +Gx1[76];
Gx2[77] = +Gx1[77];
Gx2[78] = 0.0;
;
Gx2[79] = 0.0;
;
Gx2[80] = 0.0;
;
Gx2[81] = 0.0;
;
Gx2[82] = 0.0;
;
Gx2[83] = 0.0;
;
Gx2[84] = 0.0;
;
Gx2[85] = 0.0;
;
Gx2[86] = 0.0;
;
Gx2[87] = 0.0;
;
Gx2[88] = 0.0;
;
Gx2[89] = 0.0;
;
Gx2[90] = 0.0;
;
Gx2[91] = + (real_t)5.0000000000000000e+00*Gx1[91];
Gx2[92] = + (real_t)5.0000000000000000e+00*Gx1[92];
Gx2[93] = + (real_t)5.0000000000000000e+00*Gx1[93];
Gx2[94] = + (real_t)5.0000000000000000e+00*Gx1[94];
Gx2[95] = + (real_t)5.0000000000000000e+00*Gx1[95];
Gx2[96] = + (real_t)5.0000000000000000e+00*Gx1[96];
Gx2[97] = + (real_t)5.0000000000000000e+00*Gx1[97];
Gx2[98] = + (real_t)5.0000000000000000e+00*Gx1[98];
Gx2[99] = + (real_t)5.0000000000000000e+00*Gx1[99];
Gx2[100] = + (real_t)5.0000000000000000e+00*Gx1[100];
Gx2[101] = + (real_t)5.0000000000000000e+00*Gx1[101];
Gx2[102] = + (real_t)5.0000000000000000e+00*Gx1[102];
Gx2[103] = + (real_t)5.0000000000000000e+00*Gx1[103];
Gx2[104] = + (real_t)5.0000000000000000e+00*Gx1[104];
Gx2[105] = + (real_t)5.0000000000000000e+00*Gx1[105];
Gx2[106] = + (real_t)5.0000000000000000e+00*Gx1[106];
Gx2[107] = + (real_t)5.0000000000000000e+00*Gx1[107];
Gx2[108] = + (real_t)5.0000000000000000e+00*Gx1[108];
Gx2[109] = + (real_t)5.0000000000000000e+00*Gx1[109];
Gx2[110] = + (real_t)5.0000000000000000e+00*Gx1[110];
Gx2[111] = + (real_t)5.0000000000000000e+00*Gx1[111];
Gx2[112] = + (real_t)5.0000000000000000e+00*Gx1[112];
Gx2[113] = + (real_t)5.0000000000000000e+00*Gx1[113];
Gx2[114] = + (real_t)5.0000000000000000e+00*Gx1[114];
Gx2[115] = + (real_t)5.0000000000000000e+00*Gx1[115];
Gx2[116] = + (real_t)5.0000000000000000e+00*Gx1[116];
Gx2[117] = + (real_t)2.0000000000000000e+02*Gx1[117];
Gx2[118] = + (real_t)2.0000000000000000e+02*Gx1[118];
Gx2[119] = + (real_t)2.0000000000000000e+02*Gx1[119];
Gx2[120] = + (real_t)2.0000000000000000e+02*Gx1[120];
Gx2[121] = + (real_t)2.0000000000000000e+02*Gx1[121];
Gx2[122] = + (real_t)2.0000000000000000e+02*Gx1[122];
Gx2[123] = + (real_t)2.0000000000000000e+02*Gx1[123];
Gx2[124] = + (real_t)2.0000000000000000e+02*Gx1[124];
Gx2[125] = + (real_t)2.0000000000000000e+02*Gx1[125];
Gx2[126] = + (real_t)2.0000000000000000e+02*Gx1[126];
Gx2[127] = + (real_t)2.0000000000000000e+02*Gx1[127];
Gx2[128] = + (real_t)2.0000000000000000e+02*Gx1[128];
Gx2[129] = + (real_t)2.0000000000000000e+02*Gx1[129];
Gx2[130] = +Gx1[130];
Gx2[131] = +Gx1[131];
Gx2[132] = +Gx1[132];
Gx2[133] = +Gx1[133];
Gx2[134] = +Gx1[134];
Gx2[135] = +Gx1[135];
Gx2[136] = +Gx1[136];
Gx2[137] = +Gx1[137];
Gx2[138] = +Gx1[138];
Gx2[139] = +Gx1[139];
Gx2[140] = +Gx1[140];
Gx2[141] = +Gx1[141];
Gx2[142] = +Gx1[142];
Gx2[143] = +Gx1[143];
Gx2[144] = +Gx1[144];
Gx2[145] = +Gx1[145];
Gx2[146] = +Gx1[146];
Gx2[147] = +Gx1[147];
Gx2[148] = +Gx1[148];
Gx2[149] = +Gx1[149];
Gx2[150] = +Gx1[150];
Gx2[151] = +Gx1[151];
Gx2[152] = +Gx1[152];
Gx2[153] = +Gx1[153];
Gx2[154] = +Gx1[154];
Gx2[155] = +Gx1[155];
Gx2[156] = +Gx1[156];
Gx2[157] = +Gx1[157];
Gx2[158] = +Gx1[158];
Gx2[159] = +Gx1[159];
Gx2[160] = +Gx1[160];
Gx2[161] = +Gx1[161];
Gx2[162] = +Gx1[162];
Gx2[163] = +Gx1[163];
Gx2[164] = +Gx1[164];
Gx2[165] = +Gx1[165];
Gx2[166] = +Gx1[166];
Gx2[167] = +Gx1[167];
Gx2[168] = +Gx1[168];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)2.0000000000000000e+02*Gu1[0];
Gu2[1] = + (real_t)2.0000000000000000e+02*Gu1[1];
Gu2[2] = + (real_t)2.0000000000000000e+02*Gu1[2];
Gu2[3] = + (real_t)2.0000000000000000e+02*Gu1[3];
Gu2[4] = + (real_t)2.0000000000000000e+02*Gu1[4];
Gu2[5] = + (real_t)2.0000000000000000e+02*Gu1[5];
Gu2[6] = + (real_t)2.0000000000000000e+02*Gu1[6];
Gu2[7] = + (real_t)2.0000000000000000e+02*Gu1[7];
Gu2[8] = + (real_t)3.0000000000000000e+02*Gu1[8];
Gu2[9] = + (real_t)3.0000000000000000e+02*Gu1[9];
Gu2[10] = + (real_t)3.0000000000000000e+02*Gu1[10];
Gu2[11] = + (real_t)3.0000000000000000e+02*Gu1[11];
Gu2[12] = +Gu1[12];
Gu2[13] = +Gu1[13];
Gu2[14] = +Gu1[14];
Gu2[15] = +Gu1[15];
Gu2[16] = +Gu1[16];
Gu2[17] = +Gu1[17];
Gu2[18] = +Gu1[18];
Gu2[19] = +Gu1[19];
Gu2[20] = +Gu1[20];
Gu2[21] = +Gu1[21];
Gu2[22] = +Gu1[22];
Gu2[23] = +Gu1[23];
Gu2[24] = 0.0;
;
Gu2[25] = 0.0;
;
Gu2[26] = 0.0;
;
Gu2[27] = 0.0;
;
Gu2[28] = + (real_t)5.0000000000000000e+00*Gu1[28];
Gu2[29] = + (real_t)5.0000000000000000e+00*Gu1[29];
Gu2[30] = + (real_t)5.0000000000000000e+00*Gu1[30];
Gu2[31] = + (real_t)5.0000000000000000e+00*Gu1[31];
Gu2[32] = + (real_t)5.0000000000000000e+00*Gu1[32];
Gu2[33] = + (real_t)5.0000000000000000e+00*Gu1[33];
Gu2[34] = + (real_t)5.0000000000000000e+00*Gu1[34];
Gu2[35] = + (real_t)5.0000000000000000e+00*Gu1[35];
Gu2[36] = + (real_t)2.0000000000000000e+02*Gu1[36];
Gu2[37] = + (real_t)2.0000000000000000e+02*Gu1[37];
Gu2[38] = + (real_t)2.0000000000000000e+02*Gu1[38];
Gu2[39] = + (real_t)2.0000000000000000e+02*Gu1[39];
Gu2[40] = +Gu1[40];
Gu2[41] = +Gu1[41];
Gu2[42] = +Gu1[42];
Gu2[43] = +Gu1[43];
Gu2[44] = +Gu1[44];
Gu2[45] = +Gu1[45];
Gu2[46] = +Gu1[46];
Gu2[47] = +Gu1[47];
Gu2[48] = +Gu1[48];
Gu2[49] = +Gu1[49];
Gu2[50] = +Gu1[50];
Gu2[51] = +Gu1[51];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 23, 24, 25, 36, 37, 38, 49, 50, 51, 62, 63, 64, 75, 76, 77, 88, 89, 90, 101, 102, 103, 114, 115, 116, 127, 128, 129, 140, 141, 142, 153, 154, 155, 166, 167, 168, 179, 180, 181, 192, 193, 194, 205, 206, 207, 218, 219, 220, 231, 232, 233, 244, 245, 246, 257, 258, 259, 270, 271, 272 };
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 52 ]), &(acadoWorkspace.E[ lRun3 * 52 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (13)) * (13)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (13)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (13)) * (4)) + (0) ]) );
}

acado_multQN1Gu( &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (13)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 52 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 169 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (13)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.evGu[ lRun2 * 52 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun1 = 0; lRun1 < 260; ++lRun1)
acadoWorkspace.sbar[lRun1 + 13] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = - acadoVariables.u[0];
acadoWorkspace.lb[1] = - acadoVariables.u[1];
acadoWorkspace.lb[2] = - acadoVariables.u[2];
acadoWorkspace.lb[3] = - acadoVariables.u[3];
acadoWorkspace.lb[4] = - acadoVariables.u[4];
acadoWorkspace.lb[5] = - acadoVariables.u[5];
acadoWorkspace.lb[6] = - acadoVariables.u[6];
acadoWorkspace.lb[7] = - acadoVariables.u[7];
acadoWorkspace.lb[8] = - acadoVariables.u[8];
acadoWorkspace.lb[9] = - acadoVariables.u[9];
acadoWorkspace.lb[10] = - acadoVariables.u[10];
acadoWorkspace.lb[11] = - acadoVariables.u[11];
acadoWorkspace.lb[12] = - acadoVariables.u[12];
acadoWorkspace.lb[13] = - acadoVariables.u[13];
acadoWorkspace.lb[14] = - acadoVariables.u[14];
acadoWorkspace.lb[15] = - acadoVariables.u[15];
acadoWorkspace.lb[16] = - acadoVariables.u[16];
acadoWorkspace.lb[17] = - acadoVariables.u[17];
acadoWorkspace.lb[18] = - acadoVariables.u[18];
acadoWorkspace.lb[19] = - acadoVariables.u[19];
acadoWorkspace.lb[20] = - acadoVariables.u[20];
acadoWorkspace.lb[21] = - acadoVariables.u[21];
acadoWorkspace.lb[22] = - acadoVariables.u[22];
acadoWorkspace.lb[23] = - acadoVariables.u[23];
acadoWorkspace.lb[24] = - acadoVariables.u[24];
acadoWorkspace.lb[25] = - acadoVariables.u[25];
acadoWorkspace.lb[26] = - acadoVariables.u[26];
acadoWorkspace.lb[27] = - acadoVariables.u[27];
acadoWorkspace.lb[28] = - acadoVariables.u[28];
acadoWorkspace.lb[29] = - acadoVariables.u[29];
acadoWorkspace.lb[30] = - acadoVariables.u[30];
acadoWorkspace.lb[31] = - acadoVariables.u[31];
acadoWorkspace.lb[32] = - acadoVariables.u[32];
acadoWorkspace.lb[33] = - acadoVariables.u[33];
acadoWorkspace.lb[34] = - acadoVariables.u[34];
acadoWorkspace.lb[35] = - acadoVariables.u[35];
acadoWorkspace.lb[36] = - acadoVariables.u[36];
acadoWorkspace.lb[37] = - acadoVariables.u[37];
acadoWorkspace.lb[38] = - acadoVariables.u[38];
acadoWorkspace.lb[39] = - acadoVariables.u[39];
acadoWorkspace.lb[40] = - acadoVariables.u[40];
acadoWorkspace.lb[41] = - acadoVariables.u[41];
acadoWorkspace.lb[42] = - acadoVariables.u[42];
acadoWorkspace.lb[43] = - acadoVariables.u[43];
acadoWorkspace.lb[44] = - acadoVariables.u[44];
acadoWorkspace.lb[45] = - acadoVariables.u[45];
acadoWorkspace.lb[46] = - acadoVariables.u[46];
acadoWorkspace.lb[47] = - acadoVariables.u[47];
acadoWorkspace.lb[48] = - acadoVariables.u[48];
acadoWorkspace.lb[49] = - acadoVariables.u[49];
acadoWorkspace.lb[50] = - acadoVariables.u[50];
acadoWorkspace.lb[51] = - acadoVariables.u[51];
acadoWorkspace.lb[52] = - acadoVariables.u[52];
acadoWorkspace.lb[53] = - acadoVariables.u[53];
acadoWorkspace.lb[54] = - acadoVariables.u[54];
acadoWorkspace.lb[55] = - acadoVariables.u[55];
acadoWorkspace.lb[56] = - acadoVariables.u[56];
acadoWorkspace.lb[57] = - acadoVariables.u[57];
acadoWorkspace.lb[58] = - acadoVariables.u[58];
acadoWorkspace.lb[59] = - acadoVariables.u[59];
acadoWorkspace.lb[60] = - acadoVariables.u[60];
acadoWorkspace.lb[61] = - acadoVariables.u[61];
acadoWorkspace.lb[62] = - acadoVariables.u[62];
acadoWorkspace.lb[63] = - acadoVariables.u[63];
acadoWorkspace.lb[64] = - acadoVariables.u[64];
acadoWorkspace.lb[65] = - acadoVariables.u[65];
acadoWorkspace.lb[66] = - acadoVariables.u[66];
acadoWorkspace.lb[67] = - acadoVariables.u[67];
acadoWorkspace.lb[68] = - acadoVariables.u[68];
acadoWorkspace.lb[69] = - acadoVariables.u[69];
acadoWorkspace.lb[70] = - acadoVariables.u[70];
acadoWorkspace.lb[71] = - acadoVariables.u[71];
acadoWorkspace.lb[72] = - acadoVariables.u[72];
acadoWorkspace.lb[73] = - acadoVariables.u[73];
acadoWorkspace.lb[74] = - acadoVariables.u[74];
acadoWorkspace.lb[75] = - acadoVariables.u[75];
acadoWorkspace.lb[76] = - acadoVariables.u[76];
acadoWorkspace.lb[77] = - acadoVariables.u[77];
acadoWorkspace.lb[78] = - acadoVariables.u[78];
acadoWorkspace.lb[79] = - acadoVariables.u[79];
acadoWorkspace.ub[0] = (real_t)8.5000000000000000e+00 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)8.5000000000000000e+00 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)8.5000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)8.5000000000000000e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)8.5000000000000000e+00 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)8.5000000000000000e+00 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)8.5000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)8.5000000000000000e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)8.5000000000000000e+00 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)8.5000000000000000e+00 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)8.5000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)8.5000000000000000e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)8.5000000000000000e+00 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)8.5000000000000000e+00 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)8.5000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)8.5000000000000000e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)8.5000000000000000e+00 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)8.5000000000000000e+00 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)8.5000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)8.5000000000000000e+00 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)8.5000000000000000e+00 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)8.5000000000000000e+00 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)8.5000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)8.5000000000000000e+00 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)8.5000000000000000e+00 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)8.5000000000000000e+00 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)8.5000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)8.5000000000000000e+00 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)8.5000000000000000e+00 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)8.5000000000000000e+00 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)8.5000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)8.5000000000000000e+00 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)8.5000000000000000e+00 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)8.5000000000000000e+00 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)8.5000000000000000e+00 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)8.5000000000000000e+00 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)8.5000000000000000e+00 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)8.5000000000000000e+00 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)8.5000000000000000e+00 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)8.5000000000000000e+00 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)8.5000000000000000e+00 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)8.5000000000000000e+00 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)8.5000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)8.5000000000000000e+00 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)8.5000000000000000e+00 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)8.5000000000000000e+00 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)8.5000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)8.5000000000000000e+00 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)8.5000000000000000e+00 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)8.5000000000000000e+00 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)8.5000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)8.5000000000000000e+00 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)8.5000000000000000e+00 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)8.5000000000000000e+00 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)8.5000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)8.5000000000000000e+00 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)8.5000000000000000e+00 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)8.5000000000000000e+00 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)8.5000000000000000e+00 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)8.5000000000000000e+00 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)8.5000000000000000e+00 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)8.5000000000000000e+00 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)8.5000000000000000e+00 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)8.5000000000000000e+00 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)8.5000000000000000e+00 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)8.5000000000000000e+00 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)8.5000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)8.5000000000000000e+00 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)8.5000000000000000e+00 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)8.5000000000000000e+00 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)8.5000000000000000e+00 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)8.5000000000000000e+00 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)8.5000000000000000e+00 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)8.5000000000000000e+00 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)8.5000000000000000e+00 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)8.5000000000000000e+00 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)8.5000000000000000e+00 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)8.5000000000000000e+00 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)8.5000000000000000e+00 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)8.5000000000000000e+00 - acadoVariables.u[79];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 13;
lRun4 = ((lRun3) / (13)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 39)) / (2)) + (lRun4)) - (1)) * (13)) + ((lRun3) % (13));
acadoWorkspace.A[(lRun1 * 80) + (lRun2 * 4)] = acadoWorkspace.E[lRun5 * 4];
acadoWorkspace.A[(lRun1 * 80) + (lRun2 * 4 + 1)] = acadoWorkspace.E[lRun5 * 4 + 1];
acadoWorkspace.A[(lRun1 * 80) + (lRun2 * 4 + 2)] = acadoWorkspace.E[lRun5 * 4 + 2];
acadoWorkspace.A[(lRun1 * 80) + (lRun2 * 4 + 3)] = acadoWorkspace.E[lRun5 * 4 + 3];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
acadoWorkspace.Dx0[12] = acadoVariables.x0[12] - acadoVariables.x[12];
for (lRun1 = 0; lRun1 < 320; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];
acadoWorkspace.DyN[10] -= acadoVariables.yN[10];
acadoWorkspace.DyN[11] -= acadoVariables.yN[11];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.g[ 76 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 13 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 26 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 65 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 91 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 130 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 143 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.QDy[ 169 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 182 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 195 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.QDy[ 208 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.QDy[ 221 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.QDy[ 247 ]) );

acadoWorkspace.QDy[260] = + (real_t)2.0000000000000000e+02*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[261] = + (real_t)2.0000000000000000e+02*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[262] = + (real_t)3.0000000000000000e+02*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[263] = +acadoWorkspace.DyN[3];
acadoWorkspace.QDy[264] = +acadoWorkspace.DyN[4];
acadoWorkspace.QDy[265] = +acadoWorkspace.DyN[5];
acadoWorkspace.QDy[266] = 0.0;
;
acadoWorkspace.QDy[267] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[268] = + (real_t)5.0000000000000000e+00*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[269] = + (real_t)2.0000000000000000e+02*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[270] = +acadoWorkspace.DyN[9];
acadoWorkspace.QDy[271] = +acadoWorkspace.DyN[10];
acadoWorkspace.QDy[272] = +acadoWorkspace.DyN[11];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 13 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 169 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 338 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 507 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 676 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 845 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1014 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1183 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1352 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1521 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1690 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 143 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1859 ]), &(acadoWorkspace.sbar[ 143 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2028 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 169 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2197 ]), &(acadoWorkspace.sbar[ 169 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2366 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2535 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 208 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2704 ]), &(acadoWorkspace.sbar[ 208 ]), &(acadoWorkspace.sbar[ 221 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2873 ]), &(acadoWorkspace.sbar[ 221 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3042 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 247 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3211 ]), &(acadoWorkspace.sbar[ 247 ]), &(acadoWorkspace.sbar[ 260 ]) );

acadoWorkspace.w1[0] = + (real_t)2.0000000000000000e+02*acadoWorkspace.sbar[260] + acadoWorkspace.QDy[260];
acadoWorkspace.w1[1] = + (real_t)2.0000000000000000e+02*acadoWorkspace.sbar[261] + acadoWorkspace.QDy[261];
acadoWorkspace.w1[2] = + (real_t)3.0000000000000000e+02*acadoWorkspace.sbar[262] + acadoWorkspace.QDy[262];
acadoWorkspace.w1[3] = +acadoWorkspace.sbar[263] + acadoWorkspace.QDy[263];
acadoWorkspace.w1[4] = +acadoWorkspace.sbar[264] + acadoWorkspace.QDy[264];
acadoWorkspace.w1[5] = +acadoWorkspace.sbar[265] + acadoWorkspace.QDy[265];
acadoWorkspace.w1[6] = + acadoWorkspace.QDy[266];
acadoWorkspace.w1[7] = + (real_t)5.0000000000000000e+00*acadoWorkspace.sbar[267] + acadoWorkspace.QDy[267];
acadoWorkspace.w1[8] = + (real_t)5.0000000000000000e+00*acadoWorkspace.sbar[268] + acadoWorkspace.QDy[268];
acadoWorkspace.w1[9] = + (real_t)2.0000000000000000e+02*acadoWorkspace.sbar[269] + acadoWorkspace.QDy[269];
acadoWorkspace.w1[10] = +acadoWorkspace.sbar[270] + acadoWorkspace.QDy[270];
acadoWorkspace.w1[11] = +acadoWorkspace.sbar[271] + acadoWorkspace.QDy[271];
acadoWorkspace.w1[12] = +acadoWorkspace.sbar[272] + acadoWorkspace.QDy[272];
acado_macBTw1( &(acadoWorkspace.evGu[ 988 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3211 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 247 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 247 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 936 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3042 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 884 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2873 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 221 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 221 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 832 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2704 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 208 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 208 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 780 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2535 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 195 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 195 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 728 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2366 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 182 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 182 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 676 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2197 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 169 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 169 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2028 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 572 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1859 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 143 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 143 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 520 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1690 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 130 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 130 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1521 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 416 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1352 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 364 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1183 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 91 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 91 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 312 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1014 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 260 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 845 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 65 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 65 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 208 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 676 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 52 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 507 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 104 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 338 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 26 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 26 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 52 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 169 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 13 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 13 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[0] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[1] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[2] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[3] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[4] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[5] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[6] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[50] + acadoVariables.x[50];
acadoWorkspace.lbA[7] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[8] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[9] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[10] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[11] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[11] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[12] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[13] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[77] + acadoVariables.x[77];
acadoWorkspace.lbA[14] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[15] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[15] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[16] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[90] + acadoVariables.x[90];
acadoWorkspace.lbA[17] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[17] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[101] + acadoVariables.x[101];
acadoWorkspace.lbA[18] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[19] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[20] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[20] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[114] + acadoVariables.x[114];
acadoWorkspace.lbA[21] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[21] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[115] + acadoVariables.x[115];
acadoWorkspace.lbA[22] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[22] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[23] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[23] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[127] + acadoVariables.x[127];
acadoWorkspace.lbA[24] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[24] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[128] + acadoVariables.x[128];
acadoWorkspace.lbA[25] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[25] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[129] + acadoVariables.x[129];
acadoWorkspace.lbA[26] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[26] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[140] + acadoVariables.x[140];
acadoWorkspace.lbA[27] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[27] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[141] + acadoVariables.x[141];
acadoWorkspace.lbA[28] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[28] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[142] + acadoVariables.x[142];
acadoWorkspace.lbA[29] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[29] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[153] + acadoVariables.x[153];
acadoWorkspace.lbA[30] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[30] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[154] + acadoVariables.x[154];
acadoWorkspace.lbA[31] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[31] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[155] + acadoVariables.x[155];
acadoWorkspace.lbA[32] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[32] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[166] + acadoVariables.x[166];
acadoWorkspace.lbA[33] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[33] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[167] + acadoVariables.x[167];
acadoWorkspace.lbA[34] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[34] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[168] + acadoVariables.x[168];
acadoWorkspace.lbA[35] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[35] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[179] + acadoVariables.x[179];
acadoWorkspace.lbA[36] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[36] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[180] + acadoVariables.x[180];
acadoWorkspace.lbA[37] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[37] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[181] + acadoVariables.x[181];
acadoWorkspace.lbA[38] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[38] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[192] + acadoVariables.x[192];
acadoWorkspace.lbA[39] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[39] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[193] + acadoVariables.x[193];
acadoWorkspace.lbA[40] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[40] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[194] + acadoVariables.x[194];
acadoWorkspace.lbA[41] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[41] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[205] + acadoVariables.x[205];
acadoWorkspace.lbA[42] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[42] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[206] + acadoVariables.x[206];
acadoWorkspace.lbA[43] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[43] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[207] + acadoVariables.x[207];
acadoWorkspace.lbA[44] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[44] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[218] + acadoVariables.x[218];
acadoWorkspace.lbA[45] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[45] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[219] + acadoVariables.x[219];
acadoWorkspace.lbA[46] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[46] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[220] + acadoVariables.x[220];
acadoWorkspace.lbA[47] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[47] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[231] + acadoVariables.x[231];
acadoWorkspace.lbA[48] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[48] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[232] + acadoVariables.x[232];
acadoWorkspace.lbA[49] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[49] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[233] + acadoVariables.x[233];
acadoWorkspace.lbA[50] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[50] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[244] + acadoVariables.x[244];
acadoWorkspace.lbA[51] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[51] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[245] + acadoVariables.x[245];
acadoWorkspace.lbA[52] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[52] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[246] + acadoVariables.x[246];
acadoWorkspace.lbA[53] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[53] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[257] + acadoVariables.x[257];
acadoWorkspace.lbA[54] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[54] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[258] + acadoVariables.x[258];
acadoWorkspace.lbA[55] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[55] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[259] + acadoVariables.x[259];
acadoWorkspace.lbA[56] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[56] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[270] + acadoVariables.x[270];
acadoWorkspace.lbA[57] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[57] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[271] + acadoVariables.x[271];
acadoWorkspace.lbA[58] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[58] = (real_t)3.4900000000000002e+00 - tmp;
tmp = acadoWorkspace.sbar[272] + acadoVariables.x[272];
acadoWorkspace.lbA[59] = (real_t)-3.4900000000000002e+00 - tmp;
acadoWorkspace.ubA[59] = (real_t)3.4900000000000002e+00 - tmp;

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
for (lRun1 = 0; lRun1 < 260; ++lRun1)
acadoWorkspace.sbar[lRun1 + 13] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 13 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 169 ]), &(acadoWorkspace.evGu[ 52 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 338 ]), &(acadoWorkspace.evGu[ 104 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 507 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 676 ]), &(acadoWorkspace.evGu[ 208 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 65 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 845 ]), &(acadoWorkspace.evGu[ 260 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 65 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1014 ]), &(acadoWorkspace.evGu[ 312 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 91 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1183 ]), &(acadoWorkspace.evGu[ 364 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 91 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1352 ]), &(acadoWorkspace.evGu[ 416 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1521 ]), &(acadoWorkspace.evGu[ 468 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 130 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1690 ]), &(acadoWorkspace.evGu[ 520 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 130 ]), &(acadoWorkspace.sbar[ 143 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1859 ]), &(acadoWorkspace.evGu[ 572 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 143 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2028 ]), &(acadoWorkspace.evGu[ 624 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 169 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2197 ]), &(acadoWorkspace.evGu[ 676 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 169 ]), &(acadoWorkspace.sbar[ 182 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2366 ]), &(acadoWorkspace.evGu[ 728 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 182 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2535 ]), &(acadoWorkspace.evGu[ 780 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 208 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2704 ]), &(acadoWorkspace.evGu[ 832 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 208 ]), &(acadoWorkspace.sbar[ 221 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2873 ]), &(acadoWorkspace.evGu[ 884 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 221 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3042 ]), &(acadoWorkspace.evGu[ 936 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 247 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3211 ]), &(acadoWorkspace.evGu[ 988 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 247 ]), &(acadoWorkspace.sbar[ 260 ]) );
for (lRun1 = 0; lRun1 < 273; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 13];
acadoWorkspace.state[1] = acadoVariables.x[index * 13 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 13 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 13 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 13 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 13 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 13 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 13 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 13 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 13 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 13 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 13 + 11];
acadoWorkspace.state[12] = acadoVariables.x[index * 13 + 12];
acadoWorkspace.state[234] = acadoVariables.u[index * 4];
acadoWorkspace.state[235] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[236] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[237] = acadoVariables.u[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 13 + 13] = acadoWorkspace.state[0];
acadoVariables.x[index * 13 + 14] = acadoWorkspace.state[1];
acadoVariables.x[index * 13 + 15] = acadoWorkspace.state[2];
acadoVariables.x[index * 13 + 16] = acadoWorkspace.state[3];
acadoVariables.x[index * 13 + 17] = acadoWorkspace.state[4];
acadoVariables.x[index * 13 + 18] = acadoWorkspace.state[5];
acadoVariables.x[index * 13 + 19] = acadoWorkspace.state[6];
acadoVariables.x[index * 13 + 20] = acadoWorkspace.state[7];
acadoVariables.x[index * 13 + 21] = acadoWorkspace.state[8];
acadoVariables.x[index * 13 + 22] = acadoWorkspace.state[9];
acadoVariables.x[index * 13 + 23] = acadoWorkspace.state[10];
acadoVariables.x[index * 13 + 24] = acadoWorkspace.state[11];
acadoVariables.x[index * 13 + 25] = acadoWorkspace.state[12];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 13] = acadoVariables.x[index * 13 + 13];
acadoVariables.x[index * 13 + 1] = acadoVariables.x[index * 13 + 14];
acadoVariables.x[index * 13 + 2] = acadoVariables.x[index * 13 + 15];
acadoVariables.x[index * 13 + 3] = acadoVariables.x[index * 13 + 16];
acadoVariables.x[index * 13 + 4] = acadoVariables.x[index * 13 + 17];
acadoVariables.x[index * 13 + 5] = acadoVariables.x[index * 13 + 18];
acadoVariables.x[index * 13 + 6] = acadoVariables.x[index * 13 + 19];
acadoVariables.x[index * 13 + 7] = acadoVariables.x[index * 13 + 20];
acadoVariables.x[index * 13 + 8] = acadoVariables.x[index * 13 + 21];
acadoVariables.x[index * 13 + 9] = acadoVariables.x[index * 13 + 22];
acadoVariables.x[index * 13 + 10] = acadoVariables.x[index * 13 + 23];
acadoVariables.x[index * 13 + 11] = acadoVariables.x[index * 13 + 24];
acadoVariables.x[index * 13 + 12] = acadoVariables.x[index * 13 + 25];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[260] = xEnd[0];
acadoVariables.x[261] = xEnd[1];
acadoVariables.x[262] = xEnd[2];
acadoVariables.x[263] = xEnd[3];
acadoVariables.x[264] = xEnd[4];
acadoVariables.x[265] = xEnd[5];
acadoVariables.x[266] = xEnd[6];
acadoVariables.x[267] = xEnd[7];
acadoVariables.x[268] = xEnd[8];
acadoVariables.x[269] = xEnd[9];
acadoVariables.x[270] = xEnd[10];
acadoVariables.x[271] = xEnd[11];
acadoVariables.x[272] = xEnd[12];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[260];
acadoWorkspace.state[1] = acadoVariables.x[261];
acadoWorkspace.state[2] = acadoVariables.x[262];
acadoWorkspace.state[3] = acadoVariables.x[263];
acadoWorkspace.state[4] = acadoVariables.x[264];
acadoWorkspace.state[5] = acadoVariables.x[265];
acadoWorkspace.state[6] = acadoVariables.x[266];
acadoWorkspace.state[7] = acadoVariables.x[267];
acadoWorkspace.state[8] = acadoVariables.x[268];
acadoWorkspace.state[9] = acadoVariables.x[269];
acadoWorkspace.state[10] = acadoVariables.x[270];
acadoWorkspace.state[11] = acadoVariables.x[271];
acadoWorkspace.state[12] = acadoVariables.x[272];
if (uEnd != 0)
{
acadoWorkspace.state[234] = uEnd[0];
acadoWorkspace.state[235] = uEnd[1];
acadoWorkspace.state[236] = uEnd[2];
acadoWorkspace.state[237] = uEnd[3];
}
else
{
acadoWorkspace.state[234] = acadoVariables.u[76];
acadoWorkspace.state[235] = acadoVariables.u[77];
acadoWorkspace.state[236] = acadoVariables.u[78];
acadoWorkspace.state[237] = acadoVariables.u[79];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[260] = acadoWorkspace.state[0];
acadoVariables.x[261] = acadoWorkspace.state[1];
acadoVariables.x[262] = acadoWorkspace.state[2];
acadoVariables.x[263] = acadoWorkspace.state[3];
acadoVariables.x[264] = acadoWorkspace.state[4];
acadoVariables.x[265] = acadoWorkspace.state[5];
acadoVariables.x[266] = acadoWorkspace.state[6];
acadoVariables.x[267] = acadoWorkspace.state[7];
acadoVariables.x[268] = acadoWorkspace.state[8];
acadoVariables.x[269] = acadoWorkspace.state[9];
acadoVariables.x[270] = acadoWorkspace.state[10];
acadoVariables.x[271] = acadoWorkspace.state[11];
acadoVariables.x[272] = acadoWorkspace.state[12];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[76] = uEnd[0];
acadoVariables.u[77] = uEnd[1];
acadoVariables.u[78] = uEnd[2];
acadoVariables.u[79] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index + 80];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 16 */
real_t tmpDy[ 16 ];

/** Row vector of size: 12 */
real_t tmpDyN[ 12 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 13];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 13 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 13 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 13 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 13 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 13 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 13 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 13 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 13 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 13 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 13 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 13 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[lRun1 * 13 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[16] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 16] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 16];
acadoWorkspace.Dy[lRun1 * 16 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 16 + 1];
acadoWorkspace.Dy[lRun1 * 16 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 16 + 2];
acadoWorkspace.Dy[lRun1 * 16 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 16 + 3];
acadoWorkspace.Dy[lRun1 * 16 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 16 + 4];
acadoWorkspace.Dy[lRun1 * 16 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 16 + 5];
acadoWorkspace.Dy[lRun1 * 16 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 16 + 6];
acadoWorkspace.Dy[lRun1 * 16 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 16 + 7];
acadoWorkspace.Dy[lRun1 * 16 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 16 + 8];
acadoWorkspace.Dy[lRun1 * 16 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 16 + 9];
acadoWorkspace.Dy[lRun1 * 16 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 16 + 10];
acadoWorkspace.Dy[lRun1 * 16 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 16 + 11];
acadoWorkspace.Dy[lRun1 * 16 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 16 + 12];
acadoWorkspace.Dy[lRun1 * 16 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 16 + 13];
acadoWorkspace.Dy[lRun1 * 16 + 14] = acadoWorkspace.objValueOut[14] - acadoVariables.y[lRun1 * 16 + 14];
acadoWorkspace.Dy[lRun1 * 16 + 15] = acadoWorkspace.objValueOut[15] - acadoVariables.y[lRun1 * 16 + 15];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[260];
acadoWorkspace.objValueIn[1] = acadoVariables.x[261];
acadoWorkspace.objValueIn[2] = acadoVariables.x[262];
acadoWorkspace.objValueIn[3] = acadoVariables.x[263];
acadoWorkspace.objValueIn[4] = acadoVariables.x[264];
acadoWorkspace.objValueIn[5] = acadoVariables.x[265];
acadoWorkspace.objValueIn[6] = acadoVariables.x[266];
acadoWorkspace.objValueIn[7] = acadoVariables.x[267];
acadoWorkspace.objValueIn[8] = acadoVariables.x[268];
acadoWorkspace.objValueIn[9] = acadoVariables.x[269];
acadoWorkspace.objValueIn[10] = acadoVariables.x[270];
acadoWorkspace.objValueIn[11] = acadoVariables.x[271];
acadoWorkspace.objValueIn[12] = acadoVariables.x[272];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10] - acadoVariables.yN[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11] - acadoVariables.yN[11];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 16]*(real_t)2.0000000000000000e+02;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 16 + 1]*(real_t)2.0000000000000000e+02;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 16 + 2]*(real_t)3.0000000000000000e+02;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 16 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 16 + 4];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 16 + 5];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 16 + 6]*(real_t)5.0000000000000000e+00;
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 16 + 7]*(real_t)5.0000000000000000e+00;
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 16 + 8]*(real_t)2.0000000000000000e+02;
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 16 + 9];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 16 + 10];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 16 + 11];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 16 + 12]*(real_t)6.0000000000000000e+00;
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 16 + 13]*(real_t)6.0000000000000000e+00;
tmpDy[14] = + acadoWorkspace.Dy[lRun1 * 16 + 14]*(real_t)6.0000000000000000e+00;
tmpDy[15] = + acadoWorkspace.Dy[lRun1 * 16 + 15]*(real_t)6.0000000000000000e+00;
objVal += + acadoWorkspace.Dy[lRun1 * 16]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 16 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 16 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 16 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 16 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 16 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 16 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 16 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 16 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 16 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 16 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 16 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 16 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 16 + 13]*tmpDy[13] + acadoWorkspace.Dy[lRun1 * 16 + 14]*tmpDy[14] + acadoWorkspace.Dy[lRun1 * 16 + 15]*tmpDy[15];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)2.0000000000000000e+02;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)2.0000000000000000e+02;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)3.0000000000000000e+02;
tmpDyN[3] = + acadoWorkspace.DyN[3];
tmpDyN[4] = + acadoWorkspace.DyN[4];
tmpDyN[5] = + acadoWorkspace.DyN[5];
tmpDyN[6] = + acadoWorkspace.DyN[6]*(real_t)5.0000000000000000e+00;
tmpDyN[7] = + acadoWorkspace.DyN[7]*(real_t)5.0000000000000000e+00;
tmpDyN[8] = + acadoWorkspace.DyN[8]*(real_t)2.0000000000000000e+02;
tmpDyN[9] = + acadoWorkspace.DyN[9];
tmpDyN[10] = + acadoWorkspace.DyN[10];
tmpDyN[11] = + acadoWorkspace.DyN[11];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9] + acadoWorkspace.DyN[10]*tmpDyN[10] + acadoWorkspace.DyN[11]*tmpDyN[11];

objVal *= 0.5;
return objVal;
}

