%%
clc
clear all
close all

BEGIN_ACADO;

acadoSet('problemname','nmpc');
acadoSet('results_to_file', false);


DifferentialState     xi_x xi_y xi_z; % Position in F_I
DifferentialState     xi_dot_x xi_dot_y xi_dot_z; % Velocity in F_I
DifferentialState     q_w q_x q_y q_z; % Orientation in F_I
DifferentialState     w_x w_y w_z; % Angular rate in F_B
DifferentialState     omg_1 omg_2 omg_3 omg_4; % Angular rate in F_B


Control  omgc_1 omgc_2 omgc_3 omgc_4; % Rotors speed

% Quadrotor inertia
J1 = 2.5e-3; % Davide, 0.0820 if geometric
J2 = 2.1e-3; % Davide, 0.0845 if geometric
J3 = 4.3e-3; % Davide, 0.1377 if geometric


% load the quadrotor parameters
param.J = diag([J1, J2, J3]); % inertia
param.m = 0.75; % mass (Davide), 4.34 if geometric
param.g = 9.81; % gravity
param.l = 0.14; % arm length
beta = 56; % degree
param.sbeta = sin(deg2rad(beta)); % sin(beta)
param.cbeta = cos(deg2rad(beta)); % cos(beta)
param.cq = 2.37e-8; % torque coefficient
param.ct = 1.51e-6; % thrust coefficient
cq_ct = param.cq / param.ct;
param.G1 = [1 1 1 1;
            -param.l * param.sbeta param.l * param.sbeta param.l * param.sbeta -param.l * param.sbeta;
             param.l * param.cbeta -param.l * param.cbeta param.l * param.cbeta -param.l * param.cbeta;
             cq_ct cq_ct -cq_ct -cq_ct];
         
         
param.tau = 0.019; % [s], motor's time constant, follow the alpha = (alpha_up + alpha_down)/2 formula in "Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight"
% The data is from the same paper.

param.umin = 0; % min single rotor thrust
param.umax = 8.5; % max single rotor thrust

% parameters associated with the aerodynamic drags (see eq (9) of "A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight")
param.kdx = 0.26;
param.kdy = 0.28;
param.kdz = 0.42;
param.kh = 0.01;
         

%% Differential Equation
f = acado.DifferentialEquation();       % Set the differential equation object
    
f.linkMatlabODE('quad_eom');                 % Link a Matlab ODE
%%
%% Initial conditions
x0 = [0, 0, 0]';
v0 = [0, 0, 0]';
R0 = eye(3);
W0 = [0, 0, 0]';
omg0 = [0, 0, 0, 0]';
omgc = [0, 0, 0, 0]';

tic
%%
param.t_start = 0.0; % [s] - initial time
param.t_end = 2.0; % [s] - time horizon
param.dt = 0.1; % [s] - smaple time for NMPC
param.N = round(param.t_end/param.dt); % [int] - number of control intervals

ocp = acado.OCP(param.t_start , param.t_end , param.N);      % Set up the Optimal Control Problem (OCP)
                                                             % Start at 0s, control in 20
                                                             % intervals upto 1s

% Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
% Running cost vector consists of all states and inputs.
h = {xi_x , xi_y , xi_z , xi_dot_x ,  xi_dot_y ,  xi_dot_z, ...
     q_w ,  q_x , q_y , q_z , w_x , w_y , w_z , omg_1 ,...
     omg_2 , omg_3 , omg_4 , omgc_1 , omgc_2 , omgc_3  , omgc_4};% the LSQ-Function

hN = {xi_x , xi_y , xi_z , xi_dot_x ,  xi_dot_y ,  xi_dot_z, ...
     q_w ,  q_x , q_y , q_z , w_x , w_y , w_z , omg_1 ,...
     omg_2 , omg_3 , omg_4};% the LSQ-Function

sz = size(h); 
Q = zeros(sz(1,2), sz(1,2));

Q(1,1) = 200;   % xi_x
Q(2,2) = 200;   % xi_y
Q(3,3) = 300;   % xi_z
Q(4,4) = 1;   % xi_dot_x
Q(5,5) = 1;   % xi_dot_y
Q(6,6) = 1;   % xi_dot_z
Q(7,7) = 1;   % qw : set 0 if not working !!!
Q(8,8) = 5;   % qx
Q(9,9) = 5;   % qy
Q(10,10) = 200;  % qz
Q(11,11) = 1;  % wx
Q(12,12) = 1;  % wy
Q(13,13) = 1;   % wz
Q(14,14) = 6;   % omg_1
Q(15,15) = 6;   % omg_2 
Q(16,16) = 6;   % omg_3
Q(17,17) = 6;   % omg_4
Q(18,18) = 6;   % omg_1
Q(19,19) = 6;   % omg_2 
Q(20,20) = 6;   % omg_3
Q(21,21) = 6;   % omg_4


sz = size(hN); 
QN = zeros(sz(1,2), sz(1,2));

                    % xi_x
QN(1,1) = Q(1,1);   % xi_y
QN(2,2) = Q(2,2);   % xi_z
QN(3,3) = Q(3,3);   % xi_dot_x
QN(4,4) = Q(4,4);   % xi_dot_y
QN(5,5) = Q(5,5);   % xi_dot_z
QN(6,6) = Q(6,6);   % qw
QN(7,7) = Q(7,7);   % qx
QN(8,8) = Q(8,8);   % qy
QN(9,9) = Q(9,9);   % qz
QN(10,10) = Q(10,10);  % wx
QN(11,11) = Q(11,11);  % wy
QN(12,12) = Q(12,12);  % wz
QN(13,13) = Q(13,13);
QN(14,14) = Q(14,14);  
QN(15,15) = Q(15,15);  
QN(16,16) = Q(16,16); 
QN(17,17) = Q(17,17);

% initial vector
X0 = [x0; v0;rotm2quat(R0)'; W0; omg0; omgc ];
r = zeros(1,21);                         % The reference
rN = zeros(1,17); 

% For analysis, set references.
ocp.minimizeLSQ( Q, h, r );
ocp.minimizeLSQEndTerm( QN, hN, rN );


ocp.subjectTo( f ); 



