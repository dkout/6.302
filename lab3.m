rf = 220*10^3; r1 = 10*10^3; r2 = 200;

c1 = 0.000003;

s = tf('s');

rparc = 1/(1/r1+c1*s);

Hs = rf/(rparc+r2);

h=bodeplot(Hs);%,{1,1000});
p = getoptions(h);
p.FreqUnits='Hz';
p.FreqScale= 'log'
setoptions(h,p)
zero(Hs)
