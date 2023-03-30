function [u,cost] = NMPC_casadi(param, desired,X0)
N = param.horizon; % number of control intervals

opti = casadi.Opti(); % Optimization problem
% opti = casadi.Opti('conic'); % use in combination with qpOASES

% ---- decision variables ---------
X = opti.variable(13,N+1); % state trajectory (use quaternion)
% decompose state to position, velocity, angular velocity, and quaternion
p = X(1:3,:)';
v = X(4:6,:)';
w = X(7:9,:)';
q = X(10:13,:)';

Xd = [desired.x desired.v desired.w desired.q]';% desired state for initialization

U = opti.variable(4,N+1);   % control trajectory (thrust and moment)

quaternionError = [];
for j = 1:size(X,2)
    error = q(j,1)*desired.qInv(j,1) - q(j,2:4)*desired.qInv(j,2:4)';
    error = [error q(j,1)*desired.qInv(j,2:4)+desired.qInv(j,1)*q(j,2:4) + cross(q(j,2:4),desired.qInv(j,2:4))];
    quaternionError = [quaternionError; error];
end

% ---- objective          ---------
objectiveFcn = trace((p-desired.x)*param.Q_p*(p-desired.x)') + ...
               trace((v-desired.v)*param.Q_v*(v-desired.v)') + ...
               trace((w-desired.w)*param.Q_w*(w-desired.w)') + ...
               trace(quaternionError(:,2:4)*param.Q_q*quaternionError(:,2:4)') + ...
               trace((U'-desired.ud)*param.R*(U'-desired.ud)');

opti.minimize(objectiveFcn); % race in minimal time

% ---- dynamic constraints --------
% f = @(x,u) [x(2);u-x(2)]; % dx/dt = f(x,u)
f = @(X,u) eom_Quat_NMPC(0, X, u, param); % dx/dt = f(x,u)

dt = param.dt; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints -----------
% opti.subject_to(X(7:9)'); % angular velocity is limited
opti.subject_to(param.umin<=U<=param.umax);           % control is limited

% ---- boundary conditions --------
opti.subject_to(X(:,1)==X0);   % initial state
% opti.subject_to(speed(1)==0); % ... from stand-still 
% opti.subject_to(pos(N+1)==1); % finish line at position 1

% ---- misc. constraints  ----------
% opti.subject_to(T>=0); % Time must be positive

% ---- initial values for solver ---
opti.set_initial(X, Xd);
opti.set_initial(U, desired.ud');

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
% opti.solver('qpoases'); % use in combination with opti = casadi.Opti('conic');
sol = opti.solve();   % actual solve

u = sol.value(U(:,1));
cost = sol.value(objectiveFcn);
% ---- post-processing        ------

