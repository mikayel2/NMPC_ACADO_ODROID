function Xdot = eom_R(t, X, u, param)

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

% split to states
x = X(1:3);
v = X(4:6);
W = X(7:9);
R = reshape(X(10:18), 3, 3);

f = u(1); % total thrust
M = u(2:4); % momentum

% TODO: actuator saturation
% [f, M] = saturate_fM(f, M, param); 
% commented out for simplicity or avoid the non-differentiable issue

xdot = v;
vdot = param.g * e3 ...
    - f / m * R * e3;
Wdot = J \ (-wedge(W) * J * W + M);
Rdot = R * wedge(W);

Xdot=[xdot; vdot; Wdot; reshape(Rdot,9,1)];
end