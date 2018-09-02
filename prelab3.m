Kp = 2.0; Kd = 0.2; gamma = 20; pc = -10;
s = tf('s');
Hps = gamma * (-pc/(s-pc))*(1/s)*(1/s);
K = Kp+Kd*s;

Gs = feedback(K*Hps,1,-1)

%pole(Hps)
% bode(Hps)
% margin(Hps)
% %step()
bode(Gs)
