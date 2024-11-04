clc
close all
clear 

m = 2; %kg 
J = .05; %kg m^2
tau = .2; %sec 
l = .15; %meters 
g = 9.81; %m/s^2
D = 0; % viscous damping , air drag 
s = tf('s');
C1 = tf(1, [tau, 1]);
C2 = l / (s*J);
C3 = 1/s;

Ct = C1*C2;

Ci = tf([1.2e04 , 1.398e05, 3.585e05],[1, 4571, 0]);

FGi = Ci*C1*C2;
CLi  = feedback(FGi, 1);
step(CLi)

xlabel('Time [sec]')
ylabel('Response Amplitude')
FGo = CLi*C3;

%Co = tf( [225.3 , 8001 , 1.231e04], [ 1, 1004]);
Co = tf([7.845,  9.87], [1, 0]);
T = feedback(Co*FGo, 1);
hold on 
step(T) 
xlim([0, 4]);
title('Step Responses for HW4 Figure 2')
legend('Closed Inner Loop Step Response, phi dot = w', 'Closed Outer Loop for phi ','Location','southeast')