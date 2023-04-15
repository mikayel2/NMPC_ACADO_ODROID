% This script generates the position, velocity, rotation matrix, and
% angular velocity using the differentially flat outputs (position + yaw)
% and their derivatives of the quadrotor dynamics.
% Reference: appendix of https://arxiv.org/abs/1712.02402
% Sept. 2022, Sheng Cheng

function [p,v,R,omega,f,M] = differentialFlatnessFull(p,v,a,j,s,psi,psi_dot,psi_dotdot,param)

% inputs:
% position [3x1]: p
% velocity [3x1]: v
% acceleration [3x1]: a
% jerk [3x1]: j
% yaw [1]: psi
% yaw_rate [1]: psi_dot

% outputs:
% rotation matrix [3x3]: R
% angular velocity [3x1]: omega
% total thrust [1x1]: f
% moment vector [3x1]: M

% load parameters
m = param.m; % mass
J = param.J; % inertia

% constants
z_w = [0 0 1]';
g = 9.8;


% eq 32-33
x_c = [cos(psi) sin(psi) 0]';
y_c = [-sin(psi) cos(psi) 0]';

% eq 38-39
alpha = a + g * z_w;
beta = alpha; % in our case, d_x = d_y = 0, which leads to alpha=beta

% eq 34-36
x_b = cross(y_c,alpha) / norm(cross(y_c,alpha));
y_b = cross(beta,x_b) / norm(cross(beta,x_b));
z_b = cross(x_b,y_b);

% eq 37: rotation matrix
R = [x_b y_b z_b];

% eq 41
c = z_b' * (a + g * z_w);

% eq 69-77
% d_x = d_y = d_z = 0
B1 = c;
C1 = 0;
D1 = x_b' * j;
A2 = c;
C2 = 0;
D2 = -y_b' * j;
B3 = -y_c'*z_b;
C3 = norm(cross(y_c,z_b));
D3 = psi_dot * x_c' * x_b;

% eq 66-68: angular velocity
% omega = zeros(3,1);
omega_x = (-B1 * C2 * D3 + B1 * C3 * D2 - B3 * C1 * D2 + B3 * C2 * D1) / (A2 * (B1 * C3 - B3 * C1));
omega_y = (-C1 * D3 + C3 * D1) / (B1 * C3 - B3 * C1);
omega_z = (B1 * D3 - B3 * D1) / (B1 * C3 - B3 * C1);

omega = [omega_x omega_y omega_z]';

f = c*m; % variation of (42)

xi = zeros(3,1); % (79)

x_c_dot = psi_dot * y_c; % (93)
x_b_dot = omega_z * y_b - omega_y * z_b; % (97)

c_dot = z_b' * j; % (99)

E1 = x_b' * s - 2 * c_dot * omega_y - c * omega_x * omega_z + x_b' * xi;
E2 = -y_b' * s - 2 * c_dot * omega_x + c * omega_y * omega_z - y_b' * xi;
E3 = psi_dotdot * x_c' * x_b + 2 * psi_dot * x_c' * y_b - 2 * psi_dot * omega_y * x_c' * z_b - omega_x * omega_y * y_c' * y_b - omega_x * omega_z * y_c' * z_b;

omega_x_dot = (-B1 * C2 * E3 + B1 * C3 * E2 - B3 * C1* E2 + B3 * C2 * E1)/(A2 * (B1 * C3 - B3 * C1));
omega_y_dot = (-C1 * E3 + C3 * E1)/(B1 * C3 - B3 * C1);
omega_z_dot = (B1 * E3 - B3 * E1)/(B1 * C3 - B3 * C1);

omega_dot = [omega_x_dot omega_y_dot omega_z_dot]';
M = J * omega_dot + cross(omega,J*omega);
end
