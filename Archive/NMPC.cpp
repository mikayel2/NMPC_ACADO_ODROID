#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO
  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */

  const bool CODE_GEN = false;

 // System variables
  DifferentialState     xi_x, xi_y, xi_z; // Position in F_I
  DifferentialState     xi_dot_x, xi_dot_y, xi_dot_z; // Velocity in F_I
  DifferentialState     q_w, q_x, q_y, q_z; // Orientation in F_I
  DifferentialState     w_x, w_y, w_z; // Angular rate in F_B
 
  Control               T, tau_x, tau_y, tau_z;
  
  Function              h, hN;

  OnlineData            p_F_x, p_F_y, p_F_z;
  OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;
  // Parameters with exemplary values. These are set/overwritten at runtime.
  // Simulation param
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes

  // Inertia Matrix
  const double Iv1 = 2.5;
  const double Iv2 = 2.1;
  const double Iv3 = 4.3;

  const double IIv1 = 0.4 ;
  const double IIv2 = 0.4762;
  const double IIv3 = 0.2326;


  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double m = 0.75;      // Total mass [kg]

  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]


  // System Dynamics
  // We neglect d_tau (body torque uncertainties), and 
  // f_a (aerodynamic drag effect)
  DifferentialEquation  f;
  // Translational 
  f << dot(xi_x) ==  xi_dot_x;
  f << dot(xi_y) ==  xi_dot_y;
  f << dot(xi_z) ==  xi_dot_z;

  f << dot(xi_dot_x) ==  (2 * ( q_w * q_y + q_x * q_z ) * T)/m;
  f << dot(xi_dot_y) ==  (2 * ( q_y * q_z - q_w * q_x ) * T)/m;
  f << dot(xi_dot_z) ==  (( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T)/m - g_z;

  // Rotational
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);

  f << dot(w_x) ==  IIv1 * ( Iv2 * w_y * w_z - Iv3 * w_y * w_z) + IIv1 * tau_x;
  f << dot(w_y) ==  -IIv2 * ( Iv1 * w_x * w_z - Iv3 * w_x * w_z) + IIv2 * tau_y;
  f << dot(w_z) ==  IIv3 * ( Iv1 * w_x * w_y - Iv2 * w_x * w_y) + IIv3 * tau_z;

  
// Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
// Running cost vector consists of all states and inputs.
  h << xi_x << xi_y << xi_z
    << xi_dot_x << xi_dot_y << xi_dot_z
    << q_w << q_x << q_y << q_z
    << w_x << w_y << w_z
    << T << tau_x << tau_y << tau_z;


// End cost vector consists of all states (no inputs at last state).
  hN << xi_x << xi_y << xi_z
    << xi_dot_x << xi_dot_y << xi_dot_z
    << q_w << q_x << q_y << q_z
    << w_x << w_y << w_z;

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 200;   // xi_x
  Q(1,1) = 200;   // xi_y
  Q(2,2) = 500;   // xi_z
  Q(3,3) = 1;   // xi_dot_x
  Q(4,4) = 1;   // xi_dot_y
  Q(5,5) = 1;   // xi_dot_z
  Q(6,6) = 1;   // qw : set 0 if not working !!!
  Q(7,7) = 5;   // qx
  Q(8,8) = 5;   // qy
  Q(9,9) = 200;   // qz
  Q(10,10) = 1;  // wx
  Q(11,11) = 1;  // wy
  Q(12,12) = 1;   // wz
  Q(13,13) = 6;   // T
  Q(14,14) = 6;   // tau_x  
  Q(15,15) = 6;   // tau_y
  Q(16,16) = 6;   // tau_z

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // xi_x
  QN(1,1) = Q(1,1);   // xi_y
  QN(2,2) = Q(2,2);   // xi_z
  QN(3,3) = Q(3,3);   // xi_dot_x
  QN(4,4) = Q(4,4);   // xi_dot_y
  QN(5,5) = Q(5,5);   // xi_dot_z
  QN(6,6) = Q(6,6);   // qw
  QN(7,7) = Q(7,7);   // qx
  QN(8,8) = Q(8,8);   // qy
  QN(9,9) = Q(9,9);   // qz
  QN(10,10) = Q(10,10);  // wx
  QN(11,11) = Q(11,11);  // wy
  QN(12,12) = Q(12,12);  // wz

 // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 2.0;
  r(1) = 2.0;
  r(2) = 2.0;
  r(3) = 1.0;
  //r(10) = g_z;

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  rN(3) = r(3);


  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
  }

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(-w_max_xy <= tau_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= tau_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= tau_z <= w_max_yaw);
  ocp.subjectTo( T_min <= T <= T_max);

  ocp.setNOD(10);

  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, xi_x ==  0.0 );
    ocp.subjectTo( AT_START, xi_y ==  0.0 );
    ocp.subjectTo( AT_START, xi_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, xi_dot_x ==  0.0 );
    ocp.subjectTo( AT_START, xi_dot_y ==  0.0 );
    ocp.subjectTo( AT_START, xi_dot_z ==  0.0 );
    ocp.subjectTo( AT_START, w_x ==  0.0 );
    ocp.subjectTo( AT_START, w_y ==  0.0 );
    ocp.subjectTo( AT_START, w_z ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( xi_x,"position x" );
    window1.addSubplot( xi_y,"position y" );
    window1.addSubplot( xi_z,"position z" );
    window1.addSubplot( xi_dot_x,"verlocity x" );
    window1.addSubplot( xi_dot_y,"verlocity y" );
    window1.addSubplot( xi_dot_z,"verlocity z" );
    
    //VariablesGrid gr(3, 1);
    //gr(0,0) = xi_x;
    //gr(1,0) = xi_y;
    //gr(2,0) = xi_z;

    //GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
    //window2.addSubplot3D(gr,"verlocity z");

    GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
    window3.addSubplot( tau_x,"rotation-acc x" );
    window3.addSubplot( tau_y,"rotation-acc y" );
    window3.addSubplot( tau_z,"rotation-acc z" ); 
    window3.addSubplot( T,"Thrust" );



    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    //algorithm << window2;
    algorithm << window3;
    algorithm.solve();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }


return EXIT_SUCCESS;

}
