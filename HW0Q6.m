C = tf([9], [1]) 
G = tf([1], [1, 1])
%FG = C * G
FG = series(C, G)
%print(FG)
T = feedback(FG, 1)
%T = FG / (1+FG)
%print(T)
t_eval = linspace(0, 2, 100);
[yout, t_eval2] = step(T, t_eval )

step(T, t_eval )
ylim([0, 1])
title('Step Response in Matlab')
