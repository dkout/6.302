s=tf('s')

% CONSTANTS
Mw = .08; % mass of wheel
Mp = 1.51; % robot chassis mass (from spec, weight of AA batteries)
l = .0927; % distance b/w wheel center and robot's COG
Ip = .0085; % robot moment of inertia
r = .0422; % wheel radius (measured)
g = 9.81 % gravity

Iw = Mw*r^2/2;

% fill in!!
km = .2825; % torque constant
ke = .67859; % back EMF constant
R = 7.049; % terminal resistance

B = 2*Mw + 2*Iw/r^2 + Mp;
a = Ip*B+2*Mp*l^2*(Mw+Iw/r^2);

Em=eye(4);
Em(1,3) = l;
Em(2,4) = l;

% Am=zeros(4)
% Am(1,2)=1
% Am(2,2)=2*km*ke*(Mp*l*r-Ip-Mp*l^2)/(R*r^2*a)
% Am(2,3)=Mp^2*g*l^2/a
% Am(3,4)=1
% Am(4,2)=2*km*ke*(r*B-Mp*l)/(R*r^2*a)
% Am(4,3)=Mp*g*l*B/a

Cm=zeros(1,4);
Cm(1,3)=1;


Am = [[0 1 0 0];[0 2*km*ke*(Mp*l*r-Ip-Mp*l^2)/(R*a*r^2) g*(Mp^2)*(l^2)/a 0];[0 0 0 1];[0 2*km*ke*(r*B-Mp*l)/(R*a*r^2) Mp*g*l*B/a 0]];
Bm = [[0];[2*km*(Ip+Mp*l^2 - Mp*l*r)/(R*r*a)];[0];[2*km*(Mp*l-r*B)/(R*r*a)]];
Qp = diag([0.001, 0.0001, 10, 0.05]);
K = lqr(inv(Em)*Am,inv(Em)*Bm,Qp,.00001)

%Am = Am*inv([1 0 l 0; 0 1 0 l; 0 0 1 0; 0 0 0 1]);

% Bm=zeros(4,1)
% Bm(2,1)=2*km*(Ip+Mp*l^2-Mp*l*r)/(R*r*a)
% Bm(4,1)=2*km*(Mp*l-r*B)

% Qp = zeros(4,4);
% Qp(1,1) = .00001;
% Qp(2,2) = .0001;
% Qp(3,3) = 10;
% Qp(4,4) = .05;

% Rp(1,1) = .001;

% K = lqr(inv(Em)*Am, inv(Em)*Bm,Qp,Rp);

% our K used
%K = [-10 -33.8 1052.9 77.6]

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
initial(sys_cl, [0,0,-pi/8,0], 2);

K

