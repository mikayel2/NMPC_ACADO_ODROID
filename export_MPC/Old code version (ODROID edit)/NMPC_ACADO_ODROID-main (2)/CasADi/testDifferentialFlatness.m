% This script test the differential flatness by visualizing desired
% trajectories and their associated attitude

time = 0:0.01:10;

% figure 8
xd = @(t) sin(2*pi*t/param.terminalTime);
yd = @(t) sin(4*pi*t/param.terminalTime);
zd = @(t) t*0;

vxd = @(t) 2*pi/param.terminalTime*cos(2*pi*t/param.terminalTime);
vyd = @(t) 4*pi/param.terminalTime*cos(4*pi*t/param.terminalTime);
vzd = @(t) t*0;

axd = @(t) -(2*pi/param.terminalTime)^2*sin(2*pi*t/param.terminalTime);
ayd = @(t) -(4*pi/param.terminalTime)^2*sin(4*pi*t/param.terminalTime);
azd = @(t) t*0;

jxd = @(t) -(2*pi/param.terminalTime)^3*cos(2*pi*t/param.terminalTime);
jyd = @(t) -(4*pi/param.terminalTime)^3*cos(4*pi*t/param.terminalTime);
jzd = @(t) t*0;

sxd = @(t) (2*pi/param.terminalTime)^4*sin(2*pi*t/param.terminalTime);
syd = @(t) (4*pi/param.terminalTime)^4*sin(4*pi*t/param.terminalTime);
szd = @(t) t*0;

psid = @(t) t*0;
psi_dotd = @(t) t*0;

px = xd(time);
py = yd(time);
pz = zd(time);
p = [px; py; pz];

vx = vxd(time);
vy = vyd(time);
vz = vzd(time);
v = [vx; vy; vz];

ax = axd(time);
ay = ayd(time);
az = azd(time);
a = [ax; ay; az];

jx = jxd(time);
jy = jyd(time);
jz = jzd(time);
j = [jx; jy; jz];

psi = psid(time);
psi_dot = psi_dotd(time);

% % spiral rising
% radius = 5;
% 
% px = radius*sin(2 * pi * time);
% py = radius*(1-cos(2 * pi * time));
% pz = 0.5 * time;
% p = [px; py; pz];
% 
% vx = radius*2 * pi * cos(2 * pi * time);
% vy = radius*2 * pi * sin(2 * pi * time);
% vz = 0.5 * ones(size(time));
% v = [vx; vy; vz];
% 
% ax = radius*(2 * pi)^2 * -sin(2 * pi * time);
% ay = radius*(2 * pi)^2 * cos(2 * pi * time);
% az = zeros(size(time));
% a = [ax; ay; az];
% 
% jx = radius*(2 * pi)^3 * -cos(2 * pi * time);
% jy = radius*(2 * pi)^3 * -sin(2 * pi * time);
% jz = zeros(size(time));
% j = [jx; jy; jz];
% 
% psi = 2 * pi * time;
% psi_dot = 2 * pi * ones(size(time));



R = zeros(3,3,length(time));
omega = zeros(3,length(time));
for k = 1:length(time)
    [~,~,out1,out2] = differentialFlatness(p(:,k),v(:,k),a(:,k),j(:,k),psi(k),psi_dot(k));
    R(:,:,k) = out1;
    omega(:,k) = out2;
end

%% animation
figure;
for k = 1:length(time)
    plot3(px,py,pz,'k:');
    hold on;
    scatter3(px(k),py(k),pz(k));
    plotTransforms(p(:,k)',rotm2quat(R(:,:,k)),'FrameSize',0.5);
    title(['time = ' num2str(time(k))]);

    xlabel('x');
    ylabel('y');
    zlabel('z');

%     axis([-radius-1 radius+1 -1 2*radius+1 0 3]);

    drawnow;
    pause(0.01);

    clf;
end
