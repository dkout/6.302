% ============
% Lab 12 MATLAB code for model estimation from frequency sweep data
% Be on the lookout for ***FIX THIS!!!*** notices, where you need to add/edit
% things.
% ============

clear VARIABLES;
close all;
format short e;

% ============
% User parameters
% ============
numPoles = 3;       % Number of poles in the estimated model ***FIX THIS!!*** You'll need to pick this
numZeros = 1;       % Number of zeros in the estimated model ***FIX THIS!*** You'll need to pick this
DeltaT = 1e-3;       % Teensy sample rate
sweepfname = 'csv_values_2.csv';  % Filename of frequency sweep dataset ***FIX THIS!!*** You'll need to edit this
% ============
% Read CSV file and average sweeps together
% ============
M = csvread(sweepfname,1,0);

% Get the frequency, Magnitude and wrapped phase from the data.
% Note that there are two transfer functions, desired input to arm angle,
% and desired input to motor command.
Frequency = M(:,2);
Magg = M(:,3);
Phasee = M(:,4);
Phasee(Phasee>20) = Phasee(Phasee>20) - 360;
Magc = M(:,6);
Phasec = M(:,7);

% If you let the system take n cycles of frequency data, the lines
% below compute the average magnitude and phase over n averages.
freq = sort(unique(Frequency)); % Pull out one copy of each frequency
count = zeros(size(freq)); 
mag = zeros(size(freq));
phase = zeros(size(freq));
magcommand = zeros(size(freq));
phasecommand = zeros(size(freq));

freqspot = containers.Map;
for n = 1:size(freq)
    freqspot(num2str(freq(n)))=n;
end

for n = 1:size(Frequency)
    tempn = freqspot(num2str(Frequency(n)));
    count(tempn)=count(tempn)+1;
    mag(tempn)=mag(tempn)+ Magg(n);
    phase(tempn)=phase(tempn)+Phasee(n);
    magcommand(tempn) = magcommand(tempn)+Magc(n);
    phasecommand(tempn) = phasecommand(tempn)+Phasec(n);
end

for n = 1:size(freq)
    mag(n) = mag(n)/count(n);
    phase(n) = phase(n)/count(n);
    magcommand(n) = magcommand(n)/count(n);
    phasecommand(n) = phasecommand(n)/count(n);
end

% ============
% Convert to complex frequency response and create idfrd object
% ============
y_resp = mag.*exp(phase*pi/180*1i);   % Y is the output (angle sensor measurement)
u_resp = magcommand.*exp(phasecommand*pi/180*1i); % U is the motor command (volts from PWM)
Ymeas = idfrd(y_resp,freq*2*pi,0);    % Ymeas is the TF from Y_D to Y
Umeas = idfrd(u_resp, freq*2*pi, 0);  % Umeas is the TF from Y_D to U
Pmeas = Ymeas/Umeas;   % ***FIX THIS!!*** How can we compute the transfer function of the Plant?

figure(1); clf;

bode(Ymeas); hold on;
bode(Umeas); 
bode(Pmeas); 
hold off;
legend('Measured Y/Y_D', 'Measured U/Y_D','Measured P/U');

% ============
% Use TFEST to estimate a rational transfer function
% ============
predicted_system = tfest(Pmeas,numPoles,numZeros);
tf_predicted = tf(predicted_system);

figure(2); clf;
optBode = bodeoptions;
optBode.PhaseWrapping = 'on';
bodeplot(tf_predicted,Pmeas,optBode);
legend('Estimated Rational Y/U', 'Measured Y/U');

% ======================
% Convert plant transfer function to DT state space model
% ======================
sys_ssest = ss(predicted_system);

% Discretize
sys_ssest_d = c2d(sys_ssest, DeltaT);
Ad = sys_ssest_d.a;
Bd = sys_ssest_d.b;
Cd = sys_ssest_d.c;
Dd = sys_ssest_d.d;
Nstates = size(Ad, 1);

% Plot
figure(3); clf;
pzmap(sys_ssest);
title('Pole-Zero Map of C-T State Space Model');

% =========
% Compute the gains for state feedback: u = K*x + Ku*u_d
% =========

%Kd = zeros(1, Nstates); % FIX THIS!! You'll need to find a way to compute Kd
Q = diag([1.0,100.0,10000.0]);  
R = 1.0;
Kd = dlqr(Ad,Bd,Q,R); 
% Compute closed-loop state-space matrices
Adcl = Ad - Bd*Kd;
Krd = 1.0/(Cd*((eye(size(Adcl))-Adcl) \ Bd));
Bdcl = Bd*Krd; % B*Kr=B*inv(KrInv)

% Create a closed loop state-space system
sys_cl_d = ss(Adcl, Bdcl, Cd, Dd, DeltaT);
abs(pole(sys_cl_d))

% =========
% Compute the gains for the observer system 
% xhat(i+1) = Ad*xhat(i) + Bd*u + Ld*(y-yhat)
% =========

%Ld = zeros(Nstates,1); % ***FIX THIS!!!*** You'll need to pick the observer gains
Ld = place(Ad',Cd', eig(Adcl)*0.9)';
% =========
% Simulation for three seconds at 1ms dt.
% =========
% =========
% Simulation
% =========
numSteps = 3000;  % Time interval equals numSteps * DeltaT
in = zeros(numSteps,1);
in(floor(numSteps/2):end) = 1.0;
xSave = zeros(size(Ad,1),numSteps);
xhatSave = 0.0*xSave;
xrSave = 0.0*xSave;
uSave = zeros(numSteps,1);
urSave = zeros(numSteps,1);
t = (0:numSteps-1)*DeltaT;

% xhat are the state estimates
% x are the states using state-feedback with estimated states.
% xr are the states using state-feedback with measured states.
x = ones(size(Ad,1),1);  % Initial condition on real state
xHat = 0.8*x;    % Initial condition on estimated state
xr = x; % Initial conditions of measured-state feedback state

for i = 1:numSteps-1
    
    % Micro computes: control, u, and output estimate, yhat.
    u = Krd*in(i) - Kd*xHat;   % Output update with estimated state
    yhat = Cd*xHat; %Output prediction update
    
    % Measure the output of the physical systemn
    y = Cd*x;  % Measurement of simulated physical system
    
    % Save data for plotting later.
    uSave(i) = u;
    xSave(:,i) = x;
    xhatSave(:,i) = xHat;

    
    % Micro updates the state estimation.
    xHat = Ad*xHat + Bd*u + Ld*(y-yhat); %Observer update
    
    % Simulation of plant (not a mcirocontroller computation)
    x = Ad*x + Bd*u;   % Plant update with estimated-state control
    
    % Measured-state feedback, saved for comparison purposes.
    xrSave(:,i) = xr;
    ur = Krd*in(i) - Kd*xr;  % Output update with measured ("real") state.
    urSave(i) = ur;
    xr = Ad*xr + Bd*ur;% Plant model update measured-state control

end


% Plots comparing state estimates, estimated-state feedback control, and
% measured-state feedback control.
% and 
figure(4); clf;
nPlots = size(Ad,1) + 2;
subplot(nPlots,1,1);
plot(t, Cd*xSave,'r',t, Cd*xrSave,'b',t,Cd*xhatSave,'g', t, in, 'm');
axis('tight');
title('Output: estimate-fdbk(r), meas-fdbk(b), observer-est(g), desired(m)');
subplot(nPlots,1,2);
plot(t, xSave(1,:),'r',t, xrSave(1,:),'b',t,xhatSave(1,:),'g');
axis('tight');
title('State 1: estimate-fdbk(r), meas-fdbk(b), observer-est(g)');
for i = 3:nPlots-1
    subplot(nPlots,1,i);
    plot(t, xSave(i-1,:),'r',t, xrSave(i-1,:),'b', t,xhatSave(i-1,:),'g');
    axis('tight');
    title(['State ',num2str(i-1), ': estimate-fdbk(r), meas-fdbk(b), observer-est(g)'])
end
subplot(nPlots,1,nPlots);
plot(t, uSave,'r',t, urSave,'b');
title('u(t): estimate-fdbk (r), meas-fdbk(b)')
