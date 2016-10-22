function [sys,x0,str,ts] = uav_dyn(t,x,u,flag,vx0,vyo,vzo,xo,yo,zo,w1,w2,w3,ro,po,wo,d1,d2,d3,m,jx,jy,jz)
% quadrotor w/ translation dynamics (x) w.r.t. inertial frame
% and rotation dynamics (w) w.r.t. body-frame
% with control lambda, tau and external force f_e
% the current output is y instead of x

format long;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(vx0,vyo,vzo,xo,yo,zo,w1,w2,w3,ro,po,wo);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,d1,d2,d3,m,jx,jy,jz);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,d1,d2,d3,m,jx,jy,jz);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(vx0,vyo,vzo,xo,yo,zo,w1,w2,w3,ro,po,wo)

sizes = simsizes;
sizes.NumContStates  = 18; % xdot, x, w, R
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 18; % ydot, y, w, R
sizes.NumInputs      = 7;  % lambda, tau, f_e
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

m_yaw = [cos(wo) -sin(wo) 0;sin(wo) cos(wo) 0; 0 0 1];
m_pitch = [cos(po) 0 sin(po); 0 1 0; -sin(po) 0 cos(po)];
m_roll = [1 0 0;0 cos(ro) -sin(ro);0 sin(ro) cos(ro)];

Ro = m_yaw*m_pitch*m_roll; 

x0 = [vx0,vyo,vzo,xo,yo,zo,w1,w2,w3,Ro(1,1:3),Ro(2,1:3),Ro(3,1:3)];

str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,d1,d2,d3,m,jx,jy,jz)

pdot = [x(1) x(2) x(3)]';
p = [x(4) x(5) x(6)]';
w = [x(7) x(8) x(9)]';
R = [x(10) x(11) x(12); x(13) x(14) x(15); x(16) x(17) x(18)];

% m = 0.5;
J = diag( [jx jy jz]);
d = [d1; d2; d3];

f_e = [u(1) u(2) u(3)]';
lambda = u(4);
tau = [u(5) u(6) u(7)]';


S = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];
Rdot = R*S;
tau_e = cross(d,R'*f_e);

sys = [ 1/m*(-lambda*R*[0;0;1] + m*9.81*[0;0;1] + f_e);  pdot; inv(J)*(tau + tau_e - cross(w,J*w)); Rdot(1,1:3)'; Rdot(2,1:3)'; Rdot(3,1:3)'];

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,d1,d2,d3, m,jx,jy,jz)

pdot = [x(1) x(2) x(3)]';
p = [x(4) x(5) x(6)]';
w = [x(7) x(8) x(9)]';
R = [x(10) x(11) x(12); x(13) x(14) x(15); x(16) x(17) x(18)];

S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
Rdot = R*S;

d = [d1; d2; d3];

y = p + R*d;
y_dot = pdot + Rdot*d;

sys = [y_dot; y; w; R(1,1:3)'; R(2,1:3)'; R(3,1:3)'];

% end mdlOutputs
