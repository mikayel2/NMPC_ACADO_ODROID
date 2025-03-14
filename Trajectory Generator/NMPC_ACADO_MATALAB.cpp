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
    DifferentialState q1;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState q4;
    DifferentialState Wx;
    DifferentialState Wy;
    DifferentialState Wz;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    Function acadodata_f2;
    acadodata_f2 << xx;
    acadodata_f2 << xy;
    acadodata_f2 << xz;
    acadodata_f2 << vx;
    acadodata_f2 << vy;
    acadodata_f2 << vz;
    acadodata_f2 << q2;
    acadodata_f2 << q3;
    acadodata_f2 << q4;
    acadodata_f2 << Wx;
    acadodata_f2 << Wy;
    acadodata_f2 << Wz;
    acadodata_f2 << u1;
    acadodata_f2 << u2;
    acadodata_f2 << u3;
    acadodata_f2 << u4;
    DMatrix acadodata_M1;
    acadodata_M1.read( "NMPC_ACADO_MATALAB_data_acadodata_M1.txt" );
    DVector acadodata_v1(16);
    acadodata_v1(0) = 0;
    acadodata_v1(1) = 0;
    acadodata_v1(2) = 0;
    acadodata_v1(3) = 0;
    acadodata_v1(4) = 0;
    acadodata_v1(5) = 0;
    acadodata_v1(6) = 0;
    acadodata_v1(7) = 0;
    acadodata_v1(8) = 0;
    acadodata_v1(9) = 0;
    acadodata_v1(10) = 0;
    acadodata_v1(11) = 0;
    acadodata_v1(12) = 0;
    acadodata_v1(13) = 0;
    acadodata_v1(14) = 0;
    acadodata_v1(15) = 0;
    DVector acadodata_v2(16);
    acadodata_v2(0) = 0;
    acadodata_v2(1) = 0;
    acadodata_v2(2) = 0;
    acadodata_v2(3) = 0;
    acadodata_v2(4) = 0;
    acadodata_v2(5) = 0;
    acadodata_v2(6) = 0;
    acadodata_v2(7) = 0;
    acadodata_v2(8) = 0;
    acadodata_v2(9) = 0;
    acadodata_v2(10) = 0;
    acadodata_v2(11) = 0;
    acadodata_v2(12) = 0;
    acadodata_v2(13) = 0;
    acadodata_v2(14) = 0;
    acadodata_v2(15) = 0;
    Function acadodata_f3;
    acadodata_f3 << xx;
    acadodata_f3 << xy;
    acadodata_f3 << xz;
    acadodata_f3 << vx;
    acadodata_f3 << vy;
    acadodata_f3 << vz;
    acadodata_f3 << q2;
    acadodata_f3 << q3;
    acadodata_f3 << q4;
    acadodata_f3 << Wx;
    acadodata_f3 << Wy;
    acadodata_f3 << Wz;
    DVector acadodata_v3(12);
    acadodata_v3(0) = 0;
    acadodata_v3(1) = 0;
    acadodata_v3(2) = 0;
    acadodata_v3(3) = 0;
    acadodata_v3(4) = 0;
    acadodata_v3(5) = 0;
    acadodata_v3(6) = 0;
    acadodata_v3(7) = 0;
    acadodata_v3(8) = 0;
    acadodata_v3(9) = 0;
    acadodata_v3(10) = 0;
    acadodata_v3(11) = 0;
    DMatrix acadodata_M2;
    acadodata_M2.read( "NMPC_ACADO_MATALAB_data_acadodata_M2.txt" );
    DMatrix acadodata_M3;
    acadodata_M3.read( "NMPC_ACADO_MATALAB_data_acadodata_M3.txt" );
    DVector acadodata_v4(13);
    acadodata_v4(0) = 0;
    acadodata_v4(1) = 0;
    acadodata_v4(2) = 0;
    acadodata_v4(3) = 0;
    acadodata_v4(4) = 0;
    acadodata_v4(5) = 0;
    acadodata_v4(6) = 1;
    acadodata_v4(7) = 0;
    acadodata_v4(8) = 0;
    acadodata_v4(9) = 0;
    acadodata_v4(10) = 0;
    acadodata_v4(11) = 0;
    acadodata_v4(12) = 0;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(xx) == vx;
    acadodata_f1 << dot(xy) == vy;
    acadodata_f1 << dot(xz) == vz;
    acadodata_f1 << dot(vx) == (-(2.00000000000000000000e+00*q1*q3+2.00000000000000000000e+00*q2*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01);
    acadodata_f1 << dot(vy) == (2.00000000000000000000e+00*q1*q2-2.00000000000000000000e+00*q3*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01;
    acadodata_f1 << dot(vz) == (-(-1.00000000000000000000e+00+2.00000000000000000000e+00*q1*q1+2.00000000000000000000e+00*q4*q4)*(u1+u2+u3+u4)/7.50000000000000000000e-01+9.81000000000000049738e+00);
    acadodata_f1 << dot(q1) == ((-5.00000000000000000000e-01)*Wx*q2-5.00000000000000000000e-01*Wy*q3-5.00000000000000000000e-01*Wz*q4);
    acadodata_f1 << dot(q2) == (5.00000000000000000000e-01*Wx*q1-5.00000000000000000000e-01*Wy*q4+5.00000000000000000000e-01*Wz*q3);
    acadodata_f1 << dot(q3) == (5.00000000000000000000e-01*Wx*q4+5.00000000000000000000e-01*Wy*q1-5.00000000000000000000e-01*Wz*q2);
    acadodata_f1 << dot(q4) == (-5.00000000000000000000e-01*Wx*q3+5.00000000000000000000e-01*Wy*q2+5.00000000000000000000e-01*Wz*q1);
    acadodata_f1 << dot(Wx) == (-1.16065260157705851807e-01*u1+1.16065260157705851807e-01*u2+1.16065260157705851807e-01*u3-1.16065260157705851807e-01*u4+2.09999999999999987024e-03*Wy*Wz-4.30000000000000000278e-03*Wy*Wz)/2.50000000000000005204e-03;
    acadodata_f1 << dot(Wy) == (-2.50000000000000005204e-03*Wx*Wz+4.30000000000000000278e-03*Wx*Wz+7.82870064859045589056e-02*u1-7.82870064859045589056e-02*u2+7.82870064859045589056e-02*u3-7.82870064859045589056e-02*u4)/2.09999999999999987024e-03;
    acadodata_f1 << dot(Wz) == (1.56953642384105966634e-02*u1+1.56953642384105966634e-02*u2-1.56953642384105966634e-02*u3-1.56953642384105966634e-02*u4-2.09999999999999987024e-03*Wx*Wy+2.50000000000000005204e-03*Wx*Wy)/4.30000000000000000278e-03;

    OCP ocp1(0, 1, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2, acadodata_v2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3, acadodata_v3);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u1 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u2 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u3 <= 8.50000000000000000000e+00);
    ocp1.subjectTo(0.00000000000000000000e+00 <= u4 <= 8.50000000000000000000e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wx <= 3.49000000000000021316e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wy <= 3.49000000000000021316e+00);
    ocp1.subjectTo((-3.49000000000000021316e+00) <= Wz <= 3.49000000000000021316e+00);
    ocp1.subjectTo(acadodata_f1);


    OutputFcn acadodata_f4;

    DynamicSystem dynamicsystem1( acadodata_f1,acadodata_f4 );
    Process process2( dynamicsystem1,INT_RK45 );

    RealTimeAlgorithm algo1(ocp1, 0.05);
    algo1.set( MAX_NUM_ITERATIONS, 2 );

    PeriodicReferenceTrajectory referencetrajectory(acadodata_M3);

    Controller controller3( algo1,referencetrajectory );

    SimulationEnvironment algo2(0, 1.505000E+01, process2, controller3);
     algo2.init(acadodata_v4);
    returnValue returnvalue = algo2.run();


    VariablesGrid out_processout; 
    VariablesGrid out_feedbackcontrol; 
    VariablesGrid out_feedbackparameter; 
    VariablesGrid out_states; 
    VariablesGrid out_algstates; 
    algo2.getSampledProcessOutput(out_processout);
    algo2.getProcessDifferentialStates(out_states);
    algo2.getFeedbackControl(out_feedbackcontrol);
    const char* outputFieldNames[] = {"STATES_SAMPLED", "CONTROLS", "PARAMETERS", "STATES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutSS = NULL;
    double  *outSS = NULL;
    OutSS = mxCreateDoubleMatrix( out_processout.getNumPoints(),1+out_processout.getNumValues(),mxREAL ); 
    outSS = mxGetPr( OutSS );
    for( int i=0; i<out_processout.getNumPoints(); ++i ){ 
      outSS[0*out_processout.getNumPoints() + i] = out_processout.getTime(i); 
      for( int j=0; j<out_processout.getNumValues(); ++j ){ 
        outSS[(1+j)*out_processout.getNumPoints() + i] = out_processout(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES_SAMPLED",OutSS );
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_feedbackcontrol.getNumPoints(),1+out_feedbackcontrol.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_feedbackcontrol.getNumPoints(); ++i ){ 
      outC[0*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol.getTime(i); 
      for( int j=0; j<out_feedbackcontrol.getNumValues(); ++j ){ 
        outC[(1+j)*out_feedbackcontrol.getNumPoints() + i] = out_feedbackcontrol(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_feedbackparameter.getNumPoints(),1+out_feedbackparameter.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_feedbackparameter.getNumPoints(); ++i ){ 
      outP[0*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter.getTime(i); 
      for( int j=0; j<out_feedbackparameter.getNumValues(); ++j ){ 
        outP[(1+j)*out_feedbackparameter.getNumPoints() + i] = out_feedbackparameter(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

