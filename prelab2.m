z = tf([1 0], [1], -1);
Gamma = 10; dt = 0.001; 
Hsys1 = Gamma * dt * dt * (1/(z-1))*(1/(z-1));
Kp = 1.0; Kd = 0.5; Kpd = Kp + Kd/dt * (z-1)/z;
G1 = feedback(Kpd*Hsys1,1)
% G = (Kpd*Hsys1)/(1+Kpd*Hsys1)

% rlocus(Kpd*Hsys1);
% rlocus(Kpd*Hsys1, linspace(0,10,1000));
Hsys1p = ((1-0.95)/(z-0.95))*Hsys1;
rlocus(Kpd*Hsys1, Kpd*Hsys1p, linspace(0,10,1000))
G2 = feedback(Kpd*Hsys1p,1)
Hsys1p2 = ((1-0.98)/(z-0.98))*Hsys1;
G3 = feedback(Kpd*Hsys1p2,1)