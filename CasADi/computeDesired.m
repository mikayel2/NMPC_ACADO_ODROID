function desired = computeDesired(param,currentTime)
% This function computes the value of the desired trajectory for the
% geometric control or the NMPC

% % desired trajectory (figure 8)
% xd = @(t) sin(2*pi*t/param.terminalTime);
% yd = @(t) sin(4*pi*t/param.terminalTime);
% zd = @(t) t*0;
%
% vxd = @(t) 2*pi/param.terminalTime*cos(2*pi*t/param.terminalTime);
% vyd = @(t) 4*pi/param.terminalTime*cos(4*pi*t/param.terminalTime);
% vzd = @(t) t*0;
%
% axd = @(t) -(2*pi/param.terminalTime)^2*sin(2*pi*t/param.terminalTime);
% ayd = @(t) -(4*pi/param.terminalTime)^2*sin(4*pi*t/param.terminalTime);
% azd = @(t) t*0;
%
% jxd = @(t) -(2*pi/param.terminalTime)^3*cos(2*pi*t/param.terminalTime);
% jyd = @(t) -(4*pi/param.terminalTime)^3*cos(4*pi*t/param.terminalTime);
% jzd = @(t) t*0;
%
% sxd = @(t) (2*pi/param.terminalTime)^4*sin(2*pi*t/param.terminalTime);
% syd = @(t) (4*pi/param.terminalTime)^4*sin(4*pi*t/param.terminalTime);
% szd = @(t) t*0;
%
% psid = @(t) t*0;
% psi_dotd = @(t) t*0;
% psi_dotdotd = @(t) t*0;

% % desired trajectory (3D figure 8)
% xd = @(t) sin(2*pi*t/param.terminalTime);
% yd = @(t) sin(4*pi*t/param.terminalTime);
% zd = @(t) sin(pi*t/param.terminalTime);
%
% vxd = @(t) 2*pi/param.terminalTime*cos(2*pi*t/param.terminalTime);
% vyd = @(t) 4*pi/param.terminalTime*cos(4*pi*t/param.terminalTime);
% vzd = @(t) pi/param.terminalTime*cos(pi*t/param.terminalTime);
%
% axd = @(t) -(2*pi/param.terminalTime)^2*sin(2*pi*t/param.terminalTime);
% ayd = @(t) -(4*pi/param.terminalTime)^2*sin(4*pi*t/param.terminalTime);
% azd = @(t) -(pi/param.terminalTime)^2*sin(pi*t/param.terminalTime);
%
% jxd = @(t) -(2*pi/param.terminalTime)^3*cos(2*pi*t/param.terminalTime);
% jyd = @(t) -(4*pi/param.terminalTime)^3*cos(4*pi*t/param.terminalTime);
% jzd = @(t) -(pi/param.terminalTime)^3*cos(pi*t/param.terminalTime);
%
% sxd = @(t) (2*pi/param.terminalTime)^4*sin(2*pi*t/param.terminalTime);
% syd = @(t) (4*pi/param.terminalTime)^4*sin(4*pi*t/param.terminalTime);
% szd = @(t) (pi/param.terminalTime)^4*sin(pi*t/param.terminalTime);
%
% psid = @(t) t*0;
% psi_dotd = @(t) t*0;
% psi_dotdotd = @(t) t*0;

% desired trajectory (spiral)
xd = @(t) 3*sin(4*pi*t/param.terminalTime);
yd = @(t) 3*(1-cos(4*pi*t/param.terminalTime));
zd = @(t) 3*t*0.1;

vxd = @(t) 3*4*pi/param.terminalTime*cos(4*pi*t/param.terminalTime);
vyd = @(t) 3*4*pi/param.terminalTime*sin(4*pi*t/param.terminalTime);
vzd = @(t) 3*0.1*ones(size(t));

axd = @(t) 3*-(4*pi/param.terminalTime)^2*sin(4*pi*t/param.terminalTime);
ayd = @(t) 3*(4*pi/param.terminalTime)^2*cos(4*pi*t/param.terminalTime);
azd = @(t) 3*t*0;

jxd = @(t) 3*-(4*pi/param.terminalTime)^3*cos(4*pi*t/param.terminalTime);
jyd = @(t) 3*-(4*pi/param.terminalTime)^3*sin(4*pi*t/param.terminalTime);
jzd = @(t) 3*t*0;

sxd = @(t) 3*(4*pi/param.terminalTime)^4*sin(4*pi*t/param.terminalTime);
syd = @(t) 3*-(4*pi/param.terminalTime)^4*cos(4*pi*t/param.terminalTime);
szd = @(t) 3*t*0;

psid = @(t) 4*pi/param.terminalTime*t;
psi_dotd = @(t) 4*pi/param.terminalTime*ones(size(t));
psi_dotdotd = @(t) t*0;


% the desired values for MPC contains the values for the entire
% duration
horizon = [0:param.dt:param.terminalTime+param.horizon*param.dt]';

desired.x = [xd(horizon), yd(horizon), zd(horizon)];
desired.v = [vxd(horizon), vyd(horizon), vzd(horizon)];
desired.w = [];
desired.q = [];
desired.qInv = [];

% compute the desired rotational velocity and quaternion in a for loop
ud = [];
fM = [];
for k = 1:length(horizon)
    %       [p,v,R,omega,f,M] = differentialFlatnessFull(p,v,a,j,s,psi,psi_dot,psi_dotdot,param)
    [~,~,R,omega,f,M] = differentialFlatness([xd(horizon(k)),yd(horizon(k)),zd(horizon(k))]',...
        [vxd(horizon(k)),vyd(horizon(k)),vzd(horizon(k))]',...
        [axd(horizon(k)),ayd(horizon(k)),azd(horizon(k))]',...
        [jxd(horizon(k)),jyd(horizon(k)),jzd(horizon(k))]',...
        [sxd(horizon(k)),syd(horizon(k)),szd(horizon(k))]',...
        psid(horizon(k)),psi_dotd(horizon(k)),psi_dotdotd(horizon(k)),...
        param);
    desired.w = [desired.w; omega'];

    quat = rotm2quat(R);

    if k>=2
        % make sure the generated quaternion is not flipped (rotm2quat can flip the quaternion without telling you)
        if quat*desired.q(end,:)' < 0
            quat = -quat;
        end
    end
    desired.q = [desired.q;quat];
    desired.qInv = [desired.qInv; quatinv(quat)];

    % convert totoal thrust and moment to single rotor thrust
    singleRotorInput = inv(param.G1)*[f;M];
    ud = [ud; singleRotorInput'];
    fM = [fM; [f;M]'];
end
desired.ud = ud;
desired.fM = fM;
