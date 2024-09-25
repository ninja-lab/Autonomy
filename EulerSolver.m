function [t, y] = EulerSolver(f_handle, tspan, y0, T)

%two-element vector tspan (first element is the initial time, second element is
%the final time
%output y is n columns: length of y0
%N rows - each row is time step value 

n = length(y0);
t = tspan(1):T:tspan(2);
N = floor((tspan(2)-tspan(1))/T)+1 ;
y = zeros(N, n);
y(1,:) = y0;
for k = 1:length(t)-1
    dydt = feval(f_handle, t(k), y(k,:));
    y(k + 1, :) = y(k, :) + transpose(dydt).*T ;
    k;
end
end

