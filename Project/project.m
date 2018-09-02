s=tf('s')

% CONSTANTS
Mw = 19.8447/1000; % mass of wheel
Mp = (165+23*6)/1000-2*Mw; % robot chassis mass (from spec, weight of AA batteries)
Lp = .10 % length of chassis (measured)
l = .02; % distance b/w wheel center and robot's COG
Ip = 1/12*Mp*Lp^2 + Mp*l^2; % robot moment of inertia
Iw = Mw*.037^2; % wheel moment of inertia
r = .040; % wheel radius (measured)
g = -9.81 % gravity

% fill in!!
km = 60/2/pi^2; % torque constant
ke = 1/pi; % back EMF constant
R = 20; % terminal resistance

B = 2*Mw + 2*Iw/r^2 + Mp
a = Ip*B+2*Mp*l^2*(Mw+Iw/r^2)

Em=eye(4);
Em(1,3) = -l;
Em(2,4) = -l;

% Am=zeros(4)
% Am(1,2)=1
% Am(2,2)=2*km*ke*(Mp*l*r-Ip-Mp*l^2)/(R*r^2*a)
% Am(2,3)=Mp^2*g*l^2/a
% Am(3,4)=1
% Am(4,2)=2*km*ke*(r*B-Mp*l)/(R*r^2*a)
% Am(4,3)=Mp*g*l*B/a
% 
% Bm=zeros(4,1)
% Bm(2,1)=2*km*(Ip+Mp*l^2-Mp*l*r)/(R*r*a)
% Bm(4,1)=2*km*(Mp*l-r*B)
Am = [[0 1 0 0];[0 2*km*ke*(Mp*l*r-Ip-Mp*l^2)/(R*a*r^2) g*(Mp^2)*(l^2)/a 0];[0 0 0 1];[0 2*km*ke*(r*B-Mp*l)/(R*a*r^2) Mp*g*l*B/a 0]];
Bm = [[0];[2*km*(Ip+Mp*l^2 - Mp*l*r)/(R*r*a)];[0];[2*km*(Mp*l-r*B)/(R*r*a)]];


Qp = diag([0.025,0.009, 1000, 0.6]);
Rp = 1;

K = lqr(inv(Em)*Am,inv(Em)*Bm,Qp,Rp)


Cm=zeros(1,4);
Cm(1,3)=1;
% 
% Qp = zeros(4,4);
% Qp(1,1) = .00001;
% Qp(2,2) = .0001;
% Qp(3,3) = 10;
% Qp(4,4) = .05;
% 
% Rp(1,1) = .001;
% 
% K = lqr(inv(Em)*Am, inv(Em)*Bm,Qp,Rp);

% our K used
%K = [-10 50 30 2]

Acl = Am - Bm*K;
%Kr = -inv(Cm*inv(Acl)*Bm);
Kr = 1; % don't care, since we're making it 0
Bcl = Bm*Kr

Cplot = eye(4);
Cplot(5, :) = K;

D=0;

Ts = .01 % 100 HZ

% bode(Cm*inv(s*Em-Am) * Bm)

sys_cl = dss(Acl, Bcl, Cplot, D, Em,'OutputName', {'Position', 'Velocity', 'Angle', 'Angle speed','Voltage'});
figure;
initial(sys_cl, [0,0,-pi/4,0], 5);

K

