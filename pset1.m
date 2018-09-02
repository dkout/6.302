

K=10^6;
Kc=100;
s = tf('s');
Hps = 1/(Kc*(s+1));
Gs = feedback(K*Hps,1,-1);
% Vo=Kc*Gs
% pole(Gs)
% bode(Hps)
% margin(Hps)
step(Gs)
% bode(Gs)
% rlocus(Hps)