function sys=uav_con(u)

%Disturbance in parameter and feedback
di_par = 1;
di_fee = 1;
 


%force feedback
f_e=[u(1) u(2) u(3)]'*di_fee;
if_e = [u(4) u(5) u(6)]'*di_fee;

%desired motion
y_d = [u(7) u(8) u(9)]';
y_d_dot = [u(10) u(11) u(12)]';
y_d_ddot = [u(13) u(14) u(15)]';
y_d_ddd = [u(16) u(17) u(18)]';

%desired force
f_d=[u(19)];
if_d=[u(20)];

%Motion feedback
y_dot = [u(21) u(22) u(23)]';
y = [u(24) u(25) u(26)]';
w = [u(27) u(28) u(29)]';
R = [u(30) u(31) u(32); u(33) u(34) u(35); u(36) u(37) u(38)];
y_ddot = [u(39) u(40) u(41)]';

% % assuming -pi/2 < pitch < pi/2
% theta = atan2(-R(3,1),sqrt(1-R(3,1)*R(3,1)));
% phi = atan2(R(3,2)/sqrt(1-R(3,1)^2),R(3,3)/sqrt(1-R(3,1)^2));
% psi = atan2(R(2,1)/sqrt(1-R(3,1)^2),R(1,1)/sqrt(1-R(3,1)^2));


d = [u(46);u(47);u(48)];




stp = 0.001;%step size

g = 9.81;

m = u(42);

J = diag([u(43) u(44) u(45)])*di_par;

%m=0.5*di_par;
%J = diag([0.02 0.02 0.01])*di_par;

d1 = 0.35; d2 = 0; d3 =0.3;
b =8; k =4; gamma = 0.5;
%b =5; k =5; gamma = 0.3;





%d = [d1; d2; d3];



% calculation

S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
tau_e = cross(d,R'*f_e);

%controller for motion spaces

u_c = m*y_d_ddot - b*(y_dot - y_d_dot) - k*(y - y_d);


%----------------------------------------------------------------
k_d = 30; %25  30
k_f =5; %4       2
k_i = 6; %7        6
m_a = 0.48;
eps = 0.5;%mass

k_v = 0.5;%0.5
k_p = 0.3; %0.3

%skip this
mo_con = -k_v*y_dot(1) - k_p*(y(1) - y_d(1)-0.1);%- m_a*y_ddot(1);
fo_con=-f_d+k_f*(f_e(1)-f_d)+k_i*(if_e(1)-if_d) - k_d*(y_dot(1));%+ m_a*y_ddot(1) ;

if (f_e(1)*f_e(1)>0.01)
    kkk = fo_con;
else
    kkk = mo_con;
end

%u_c=[kkk; u_c(2); u_c(3)];

%----------------------------------------------------------------
%decompose omega
k_star =-1.1*d1/d3;
u_bar = 2;

%
alpha_o = sqrt(d1*d1+d3*d3);
delta_o = [-d3/alpha_o 0 d1/alpha_o;
    0 1 0;
    d1/alpha_o 0 d3/alpha_o];


nu_l = delta_o*w;

u_hat = R'*(-u_c/m + 9.81*[0; 0; 1]);



nu_d(3) = nu_l(3);
nu_d_dot = [-u_hat(2)/alpha_o - nu_l(2)*nu_d(3) ; -u_hat(1)/d3+(nu_l(1)*nu_l(1)+nu_l(2)*nu_l(2))*d1/d3 + nu_l(1)*nu_d(3); 0];

nu_d_dot = [nu_d_dot(1); nu_d_dot(2); k_star*(nu_d_dot(1)+nu_d_dot(1)*nu_l(2)*nu_l(2)+2*nu_l(1)*nu_l(2)*nu_d_dot(2)-atan(nu_l(1))*nu_d_dot(2)-nu_l(2)/(1+nu_l(1)*nu_l(1)))];



nu_d = nu_l+nu_d_dot*stp;
nu_d(3) = k_star*nu_l(1)*(1+nu_l(2)*nu_l(2))-atan(nu_l(1))*k_star*nu_l(2)+atan(nu_l(1))*u_bar;




w_d = delta_o*nu_d;
w_d_dot = delta_o*nu_d_dot;

lambda = m*(u_hat(3) - d1*nu_d_dot(2)-d3*(nu_l(1)*nu_l(1)+nu_l(2)*nu_l(2))+d1*nu_l(1)*nu_l(3))


tau = cross(w,J*w) +J*(w_d_dot - 2*gamma*(w-w_d)) - 0.15*gamma*diag([1 1 1])*w - tau_e



sys = [lambda; tau;nu_l;nu_d;nu_d_dot];
