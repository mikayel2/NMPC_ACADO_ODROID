% This dynamics is for the NMPC solver to propagate the dynamics
% Rotor dynamcis is not considered
% Sheng Cheng, Dec 2022
function Xdot = eom_Quat_NMPC(t, X, u, param)

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

% split to states
x = X(1:3);
v = X(4:6);
W = X(7:9);
q = X(10:13);

% compute the third column of Rotation matrix based on quaternion
% reference: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
z3 = [2 * (q(2) * q(4) + q(1) * q(3));
      2 * (q(3) * q(4) - q(1) * q(2));
      2 * (q(1) * q(1) + q(4) * q(4)) - 1]; 

physcialControl = param.G1*u; % convert single rotor thrust to physical control inputs (total thrust and moment)

f = physcialControl(1); % total thrust
M = physcialControl(2:4); % momentum

% compute the aerodynamic drags
% refer to eq (9) of "A Comparative Study of Nonlinear MPC and Differential-Flatness-Based Control for Quadrotor Agile Flight"
z1 = [2*(q(1)^2 + q(2)^2) - 1;
      2 * q(2) * q(3) + 2 * q(1) * q(4);
      2 * q(2) * q(4) - 2 * q(1) * q(3)];
z2 = [2 * q(2) * q(3) - q(1) * q(4);
      2 * (q(1)^2 + q(3)^2) - 1;
      2 * q(3) * q(4) + 2 * q(1) * q(2)];
v_body = [z1 z2 z3]'*v; % convert to body-frame velocity
f_aero = [-param.kdx * v_body(1);
          -param.kdy * v_body(2);
          -param.kdz * v_body(3) + param.kh*(v_body(1)^2 + v_body(2)^2)]; % aerodynamic drags in the body frame

xdot = v;
vdot = param.g * e3 ...
    - f / m * z3 + [z1 z2 z3] * f_aero / m; % f_aero is in body frame. We need to turn it back to the inertial frame.
Wdot = J \ (-cross(W, J * W) + M);

% qdot = 0.5*quatMult(q,[0;W]); 
qdot = 0.5*[-q(2:4)'*W;
        q(1)*W  + cross(q(2:4),W)];
% formula pulled from eq. 2.24 in https://www.sciencedirect.com/science/article/pii/B9780124158429000022
Xdot=[xdot; vdot; Wdot; qdot];
end