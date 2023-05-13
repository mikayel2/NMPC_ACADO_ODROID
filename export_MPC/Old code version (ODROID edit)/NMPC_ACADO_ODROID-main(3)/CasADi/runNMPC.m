% This script runs the NMPC of quadrotor using the CASADI solver
% Sheng Cheng, Nov. 2022


%% Disturbances
% Uncomment to remove disturbances.
% param.use_disturbances = false;

%% Simulation parameters
param.dt = 0.05; % [s], smaple time for NMPC
param.sysSampleTime = 0.0025; % [s] system sample time for lower-level control
param.terminalTime = 10; % [s]
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

%% Initial conditions
x0 = [0, 0, 0]';
v0 = [0, 0, 0]';
R0 = eye(3);
W0 = [0, 0, 0]';

tic

%% controller configuration

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

%% place to store the states
X_storage = [X0];
pd_storage = [x0];
u_storage = [];
cost_storage = [];
rotorSpeed_storage = [0;0;0;0];
%% for loop here
for k = 1:N
    % get new states
    X_now = X_storage(:,end);

    % assemble the control action using NMPC
    desired.x = NMPCdesired.x(k:k+param.horizon,:);
    desired.v = NMPCdesired.v(k:k+param.horizon,:);
    desired.w = NMPCdesired.w(k:k+param.horizon,:);
    desired.q = NMPCdesired.q(k:k+param.horizon,:);
    desired.qInv = NMPCdesired.qInv(k:k+param.horizon,:);
    desired.ud = NMPCdesired.ud(k:k+param.horizon,:);

    %         cadadi implementation
    [u,cost] = NMPC_casadi(param, desired,X_now);

    u_storage = [u_storage u];
    cost_storage = [cost_storage cost];

    % put into Numerical integration for one sample interval
    timeInterval = [k-1 k]*param.sysSampleTime;
    
    t_seed = tic;
    rotorSpeed = rotorSpeed_storage(:,end);
    for j = 1:param.dt/param.sysSampleTime
        % integrate with finer dynamics that consider the rotor dynamics
        [t, X_new] = ode45(@(t, Xin) eom_Quat(t, Xin, u, param), timeInterval, [X_now;rotorSpeed], odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
        X_now = X_new(end,1:end-4)';
        rotorSpeed = X_new(end,end-3:end)';
    end
    fprintf("ode solution time is %.3f\n",toc(t_seed));

    % update state
    X_storage = [X_storage X_now];
    rotorSpeed_storage = [rotorSpeed_storage rotorSpeed];
end

toc

%% plot
% position
figure;
subplot(3,1,1);
plot(param.time,X_storage(1,:)); 
hold on;
plot(param.time,NMPCdesired.x(1:length(param.time),1)); 
ylabel('x [m]');
title('Position');

subplot(3,1,2);
plot(param.time,X_storage(2,:)); 
hold on;
plot(param.time,NMPCdesired.x(1:length(param.time),2)); 
ylabel('y [m]');

subplot(3,1,3);
plot(param.time,X_storage(3,:)); 
hold on;
plot(param.time,NMPCdesired.x(1:length(param.time),3)); 
legend('actual','desired')
ylabel('z [m]');
xlabel('time [s]');

% velocity
figure;
subplot(3,1,1);
plot(param.time,X_storage(4,:)); 
hold on;
plot(param.time,NMPCdesired.v(1:length(param.time),1)); 
ylabel('x [m/s]');
title('Velocity');

subplot(3,1,2);
plot(param.time,X_storage(5,:)); 
hold on;
plot(param.time,NMPCdesired.v(1:length(param.time),2)); 
ylabel('y [m/s]');

subplot(3,1,3);
plot(param.time,X_storage(6,:)); 
hold on;
plot(param.time,NMPCdesired.v(1:length(param.time),3)); 
legend('actual','desired')
ylabel('z [m/s]');
xlabel('time [s]');


% angular rate
figure;
subplot(3,1,1);
plot(param.time,X_storage(7,:)); 
hold on;
plot(param.time,NMPCdesired.w(1:length(param.time),1)); 
ylabel('x [rad/s]');
title('Angular rate');

subplot(3,1,2);
plot(param.time,X_storage(8,:)); 
hold on;
plot(param.time,NMPCdesired.w(1:length(param.time),2)); 
ylabel('y [rad/s]');

subplot(3,1,3);
plot(param.time,X_storage(9,:)); 
hold on;
plot(param.time,NMPCdesired.w(1:length(param.time),3)); 
legend('actual','desired')
ylabel('z [rad/s]');
xlabel('time [s]');

% quaternion
figure;
subplot(4,1,1);
plot(param.time,X_storage(10,:)); 
hold on;
plot(param.time,NMPCdesired.q(1:length(param.time),1)); 
ylabel('w');
ylim([-1 1]);
title('Quaternion');

subplot(4,1,2);
plot(param.time,X_storage(11,:)); 
hold on;
plot(param.time,NMPCdesired.q(1:length(param.time),2)); 
ylabel('x');
ylim([-1 1]);

subplot(4,1,3);
plot(param.time,X_storage(12,:)); 
hold on;
plot(param.time,NMPCdesired.q(1:length(param.time),3)); 
ylabel('y');
ylim([-1 1]);

subplot(4,1,4);
plot(param.time,X_storage(13,:)); 
hold on;
plot(param.time,NMPCdesired.q(1:length(param.time),4)); 
legend('actual','desired')
ylabel('z');
xlabel('time [s]');
ylim([-1 1]);

% rotor speed
figure;
plot(param.time,rotorSpeed_storage');
legend('m1','m2','m3','m4');