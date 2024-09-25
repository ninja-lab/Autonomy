dxdt = @(t, x) -x + exp(-2*t);
[t, x_] = ode45(dxdt, [0 10], 0);
figure;
plot(t, x_, "b-")% 'LineWidth', 4);
hold on
x = HW0Q2-analytical(t); 
plot(t, x,"r--");
ylim([0,1]);
xlabel('Time t'); ylabel('x(t)');