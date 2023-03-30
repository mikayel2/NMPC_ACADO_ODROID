function [ dx ] = ode( t,x,u,param)

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

% split to states
x = x(1:3);
v = x(4:6);
W = x(11:13);
q = x(7:10);

% compute the third column of Rotation matrix based on quaternion
z3 = [2 * (q(2) * q(4) + q(1) * q(3));
      2 * (q(3) * q(4) - q(1) * q(2));
      2 * (q(1) * q(1) + q(4) * q(4)) - 1];
  
rotorSpeed = x(14:17);
desiredSingleRotorThrust = u;
desiredSingleRotorSpeed = sqrt(desiredSingleRotorThrust / param.ct);

% the rotor speed satisfies the first-order dynamics
% see eq (8) in "Thrust Mixing, Saturation, and Body-Rate Control for
% Accurate Aggressive Quadrotor Flight"
rotorSpeeddot = 1/param.tau * (desiredSingleRotorSpeed - rotorSpeed);


actualControl = param.G1 * param.ct * rotorSpeed.^2;
f = actualControl(1); % total thrust
M = actualControl(2:4); % momentum

% actuator saturation
% [f, M] = saturate_fM(f, M, param); 
% commented out for simplicity or avoid the non-differentiable issue

% compute the aerodynamic drags
% refer to eq (9) of "A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight"
v_body = quat2rotm(q')'*v; % convert to body-frame velocity
f_aero = [-param.kdx * v_body(1);
          -param.kdy * v_body(2);
          -param.kdz * v_body(3) + param.kh*(v_body(1)^2 + v_body(2)^2)]; % aerodynamic drags in the body frame

xdot = v;
vdot = param.g * e3 ...
    - f / m * z3 + quat2rotm(q') * f_aero / m; % f_aero is in body frame. We need to turn it back to the inertial frame.
Wdot = J \ (-cross(W, J * W) + M);

% qdot = 0.5*quatMult(q,[0;W]); 
qdot = 0.5*[-q(2:4)'*W;
        q(1)*W  + cross(q(2:4),W)];
% formula pulled from eq. 2.24 in https://www.sciencedirect.com/science/article/pii/B9780124158429000022
dx =[xdot; vdot; Wdot; qdot;rotorSpeeddot];
    
%     % Debug code
%     disp('====== EVAL ODE ======');
%     disp(t)
%     disp(x)
%     disp(u)
%     disp(p)
%     disp(w)
end