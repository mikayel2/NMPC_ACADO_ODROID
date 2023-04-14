% This script runs the NMPC of quadrotor using the CASADI solver
% Sheng Cheng, Nov. 2022


% Disturbances
% Uncomment to remove disturbances.
% param.use_disturbances = false;

% Simulation parameters
param.dt = 10.9500/2628; % [s], smaple time for NMPC
param.sysSampleTime = 0.0025; % [s] system sample time for lower-level control
param.terminalTime = 11; % [s]
param.time = 0:param.dt:param.terminalTime;
param.sysTime = 0:param.sysSampleTime:param.terminalTime;
N = length(param.time)-1;

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

% Initial conditions
x0 = [0, 0, 0]';
v0 = [0, 0, 0]';
R0 = eye(3);
W0 = [0, 0, 0]';

tic

% controller configuration

param.horizon = 20; % horizon of NMPC (total of 20 * 0.05 = 1 s)
param.Q_p = diag([200,200,300]); % cost matrix for position error
param.Q_v = diag([1,1,1]); % cost matrix for velocity error
param.Q_q = diag([5,5,200]); % cost matrix for quaternion error
param.Q_w = diag([1,1,1]); % cost matrix for angular velocity error
param.R = diag([6,6,6,6]); % cost matrix for control input

% dummy parameters for computing the desired control actions
param.kx = 16*ones(3,1);
param.kv = 5.6*ones(3,1);
param.kR = 8.81*ones(3,1);
param.kW = 2.54*ones(3,1);

% initial vector
X0 = [x0; v0; W0; rotm2quat(R0)'];

% precompute the desired values
NMPCdesired = computeDesired(param,0);
% place to store the states
% X_storage = [X0];
% pd_storage = [x0];
% u_storage = [];
% cost_storage = [];
% rotorSpeed_storage = [0;0;0;0];
% for k = 1:N
% % assemble the control action using NMPC
% desired.x = NMPCdesired.x(k:k+param.horizon,:);
% desired.v = NMPCdesired.v(k:k+param.horizon,:);
% desired.w = NMPCdesired.w(k:k+param.horizon,:);
% desired.q = NMPCdesired.q(k:k+param.horizon,:);
% desired.qInv = NMPCdesired.qInv(k:k+param.horizon,:);
% desired.ud = NMPCdesired.ud(k:k+param.horizon,:);
% end