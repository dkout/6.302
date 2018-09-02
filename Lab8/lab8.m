% Script to compute propeller system using pole placement.

% Third order model of propellor system

% States: [Angle (Theta_a), 
%          Rotational velocity (Omega_a), 
%          back-EMF (deltaVemf)]

% Using a "descriptor" system:
% E dx/dt = A x + B u,
% y = Cx + Du,
% where E is 3x3, A is 3x3, B is 3x1 and C is 1x3

% Parameters
Ke = 5.5e-3;  % back-emf per radian/sec motor rotational velocity
Km = 5.5e-3;  % Torque per amp
Jm = 3e-6;  % Motor moment of inertia
Ja = 4.5e-4; % Arm moment of inertia
Rm = 1;      % Motor resistance
Rs = 1;      % Series resistance
La = 0.15;   % Arm length in meters
Kf = 10e-6;  % Motor Friction
Kt = 1.8e-3;

% E matrix, multiplies by dx/dt
E = eye(3);
E(2,2) = Ja;
E(3,3) = Jm;

% Motor DC resistance is one ohm plus a one-ohm shunt resistor (in series)
% Motor Back-EMF = (EMF generated by rotational velocity Omega)

% Dtheta/dt = Omega.
A(1,2) = 1;

% Ja d/dt omega = Kt*La/Ke Vbemf
A(2,3) = Kt*La/Ke;

% Jm d/dt Vbemf = -((Ke*Km)/(Rtotal) + Kf) Vemf + (Ke*Km)/(Rtotal) Vpwm
A(3,:) = [0 0 -(Ke*Km/(Rs+Rm)+ Kf)];
B = [0 0 Ke*Km/(Rs+Rm)]';

% Output is voltage from angle sensor, approximately one volt per radian.
C = [1 0 0];

% The instantaneous u-to-y term (D) is zero.
D = 0;

% Convert to a system
motor_sys = dss(A, B, C, D, E, 'StateName', {'Arm Angle', 'Arm Rot. Vel', 'Vemf'});
display(motor_sys);

% Print input-output transfer function.
%tf(motor_sys)

% Determine poles
disp('Poles of the Open Loop System')
pole(motor_sys)

% Bode plot of transfer function
%figure;
%bode(motor_sys);

% Use the pole placer (placer requires E=I, notice division by E)
K = place(motor_sys.E\motor_sys.A,motor_sys.E\motor_sys.B,[-100,-200,-300]);

% Compute Closed Loop Matrix
%********FIX THIS!!!!!!**********************
Acl = A-B*K;  

% Compute input scaling so that theta_d  - theta_a = 0 in steady-state.
%********FIX THIS!!!!!!**********************
Kr = -inv(C*(inv(A-B*K)*B));

% Make every state an output so they will be plotted by step command.
Cplot = eye(3); 

% Vpwm = Kr*Theta_d - K*x, make the -K*x part an output for plotting
%********FIX THIS!!!!!!**********************
Cplot(4,:) = -K; % Last output should give -K*x part of Vpwm 


% Closed Loop systen
sys_cl = dss(Acl, B*Kr, Cplot, D, E, 'OutputName', {'Arm Angle', 'Arm Rot. Vel', 'Vemf', '-K*x part of Vpwm'});
disp('Poles of the Closed-Loop System');
disp(pole(sys_cl));

% Plot step response of all three states and the -K*x part of Vpwm.
figure;
step(sys_cl);
display(K);
display(Kr);