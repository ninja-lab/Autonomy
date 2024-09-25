% Define the ODE function
dxdt = @(t, x) -x + exp(-2*t);

% Solve the ODE numerically
t_span = [0 10];
x0 = 0;
[t, x_num] = ode45(dxdt, t_span, x0);

% Compute the analytical solution
t_eval = linspace(0, 10, 100);
x_analytical = exp(-t_eval) .* (1 - exp(-t_eval));

% Plot both solutions
figure;
plot(t, x_num, 'b-', 'LineWidth', 2); % Numerical solution
hold on;
plot(t_eval, x_analytical, 'r*', 'LineWidth', 2); % Analytical solution
xlabel('Time t');
ylabel('x(t)');
legend('Numerical', 'Analytical');
title('Solution of \dot{x} = -x + e^{-2t}');
hold off;
