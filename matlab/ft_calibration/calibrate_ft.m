function output_casadi = calibrate_ft(expected_ft, measured_ft)

% Scrivi la teoria

opti = casadi.Opti();

C = opti.variable(6,6);
o = opti.variable(6,1);

num_samples = size(expected_ft,1);

shuffled_indeces = randperm(num_samples);

cost = sumsqr(expected_ft(shuffled_indeces(1),:)' - C * measured_ft(shuffled_indeces(1),:)' + o);

for i = 2 : num_samples
    cost = cost + sumsqr(expected_ft(shuffled_indeces(i),:)' - C * measured_ft(shuffled_indeces(i),:)' + o);
    i
end

cost = cost / num_samples;

lam = 5;

cost = cost + lam * sumsqr(C - eye(6));

opti.minimize(cost);

opti.solver('ipopt');
sol = opti.solve();

output_casadi.C = sol.value(C);
output_casadi.o = sol.value(o);

end