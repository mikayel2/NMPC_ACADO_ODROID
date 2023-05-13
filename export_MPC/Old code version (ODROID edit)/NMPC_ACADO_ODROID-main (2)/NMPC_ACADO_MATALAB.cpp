/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState xx;
    DifferentialState xy;
    DifferentialState xz;
    DifferentialState vx;
    DifferentialState vy;
    DifferentialState vz;
    DifferentialState Wx;
    DifferentialState Wy;
    DifferentialState Wz;
    DifferentialState q1;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState q4;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    DMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_ACADO_MATALAB_data_acadodata_M1.txt" );
    Function acadodata_f2;
    acadodata_f2 << xx;
    acadodata_f2 << xy;
    acadodata_f2 << xz;
    acadodata_f2 << vx;
    acadodata_f2 << vy;
    acadodata_f2 << vz;
    acadodata_f2 << Wx;
    acadodata_f2 << Wy;
    acadodata_f2 << Wz;
    acadodata_f2 << q2;
    acadodata_f2 << q3;
    acadodata_f2 << q4;
    acadodata_f2 << u1;
    acadodata_f2 << u2;
    acadodata_f2 << u3;
    acadodata_f2 << u4;
    DMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_ACADO_MATALAB_data_acadodata_M2.txt" );
    Function acadodata_f3;
    acadodata_f3 << xx;
    acadodata_f3 << xy;
    acadodata_f3 << xz;
    acadodata_f3 << vx;
    acadodata_f3 << vy;
    acadodata_f3 << vz;
    acadodata_f3 << Wx;
    acadodata_f3 << Wy;
    acadodata_f3 << Wz;
    acadodata_f3 << q2;
    acadodata_f3 << q3;
    acadodata_f3 << q4;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(xx) == vx;
    acadodata_f1 << dot(xy) == vy;
    acadodata_f1 << dot(xz) == vz;
    acadodata_f1 << dot(vx) == (-(2.00000000000000000000e+00*q1*q3+2.00000000000000000000e+00*q2*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01);
    acadodata_f1 << dot(vy) == (2.00000000000000000000e+00*q1*q2-2.00000000000000000000e+00*q3*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01;
    acadodata_f1 << dot(vz) == (-(-1.00000000000000000000e+00+2.00000000000000000000e+00*q1*q1+2.00000000000000000000e+00*q4*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01+9.81000000000000049738e+00);
    acadodata_f1 << dot(Wx) == (-1.16065260157705851807e-01*u1+1.16065260157705851807e-01*u2+1.16065260157705851807e-01*u3-1.16065260157705851807e-01*u4+2.09999999999999987024e-03*Wy*Wz-4.30000000000000000278e-03*Wy*Wz)/2.50000000000000005204e-03;
    acadodata_f1 << dot(Wy) == (-2.50000000000000005204e-03*Wx*Wz+4.30000000000000000278e-03*Wx*Wz+7.82870064859045589056e-02*u1-7.82870064859045589056e-02*u2+7.82870064859045589056e-02*u3-7.82870064859045589056e-02*u4)/2.09999999999999987024e-03;
    acadodata_f1 << dot(Wz) == (1.56953642384105966634e-02*u1+1.56953642384105966634e-02*u2-1.56953642384105966634e-02*u3-1.56953642384105966634e-02*u4-2.09999999999999987024e-03*Wx*Wy+2.50000000000000005204e-03*Wx*Wy)/4.30000000000000000278e-03;
    acadodata_f1 << dot(q1) == ((-5.00000000000000000000e-01)*Wx*q2-5.00000000000000000000e-01*Wy*q3-5.00000000000000000000e-01*Wz*q4);
    acadodata_f1 << dot(q2) == (5.00000000000000000000e-01*Wx*q1-5.00000000000000000000e-01*Wy*q4+5.00000000000000000000e-01*Wz*q3);
    acadodata_f1 << dot(q3) == (5.00000000000000000000e-01*Wx*q4+5.00000000000000000000e-01*Wy*q1-5.00000000000000000000e-01*Wz*q2);
    acadodata_f1 << dot(q4) == (-5.00000000000000000000e-01*Wx*q3+5.00000000000000000000e-01*Wy*q2+5.00000000000000000000e-01*Wz*q1);

    OCP ocp1(0, 1, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u1 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u2 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u3 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u4 <= 8.50000000000000000000e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wx <= 3.49000000000000021316e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wy <= 3.49000000000000021316e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wz <= 3.49000000000000021316e+00);
    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 4 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 40 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( HOTSTART_QP, NO );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

