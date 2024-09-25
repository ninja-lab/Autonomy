function x = x_analytical(t)
    x = exp(-t) .* (1 - exp(-t));
end
