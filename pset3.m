s = tf('s');
H_op = 10^6/(s*(s/(10^5)+1)*(s/(3*10^5)+1));
f=0.01;
H = H_op/(1+f*H_op);
sys = feedback(H,1);
margin(f*H_op)     