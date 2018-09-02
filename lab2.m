z = tf([1 0], [1], -1); dt = 0.001;
hw2th = dt/(z-1);
dt=0.001; kp=10; kd=2; ki=6; m=3; g=15;
P=1;
p=0.991;
h2ca = g*(1-p)/(z-p);
hw2th = dt*(1/(1-z));
ha2w = dt*(1/(1-z));

H = g*ha2w*hw2th;
Hp = h2ca*ha2w*hw2th;

pid = kp+kd/m/dt*(1-z^(-m))+ dt*ki/(1-1/z);
loop_w_poles = feedback(Hp*pid, 1);
pole(loop_w_poles)
step(loop_w_poles)
% disp(2*pi/1000/2.48-1.95);


