s = tf('s');

Hs = 10^7/((s+10)*(s+1000)*(s+1000));
K=200;

fb = K*Hs/(1+K*Hs)

margin(K*Hs)
step(fb)
pole(fb)