
t_span = [0, 2];
y0 = [0, 0, 0, 0];
T1 = .1;
[t1,yout1] = EulerSolver(@dydt, t_span, y0, T1);
T01=.00001;
[t01,yout01] = EulerSolver(@dydt, t_span, y0, T01);

function yp = dydt(t, y)
    y1 = y(1);
    y2 = y(2);
    y3 = y(3);
    y4 = y(4);
    yp1 = y4;
    yp2 = y3;
    yp3 = -y1;
    yp4 = -y2*y1+t;
    yp = [yp1;yp2;yp3;yp4]
end

% Solve the ODE numerically
[t_eval, y_num] = ode45(@dydt, t_span, y0);
plot(t_eval, y_num(:,2), 'k*', 'MarkerSize',20)

hold on
plot(t1, yout1(:,2).','r-', 'LineWidth', 3 )
plot(t01, yout01(:,2).','b-', 'LineWidth', 3 )
title('State Trajectory for x(t)', 'FontSize',28)
xlabel('Time [sec]','FontSize',22)
ylabel('Amplitude Response', 'FontSize',22)
legend('ode45', 'Fwd Euler T = .1s', 'Fwd Euler T = .01s','FontSize',22)
set(gca, 'FontSize', 24); 