function output_casadi = calibrate_ft(config, expected_ft, measured_ft)

sigma = eye(6);
if config.scaling_opt
    tmp_vec = zeros(6,1);
    for i = 1 : size(measured_ft,2)
        tmp_vec(i) = 1 / (max(measured_ft(:,i)) - min(measured_ft(:,i)));
    end
    sigma = diag(tmp_vec);
end

% Scrivi la teoria

opti = casadi.Opti();

C = opti.variable(6,6);
o = opti.variable(6,1);

num_samples = size(expected_ft,1);

shuffled_indeces = randperm(num_samples);

cost = 0;

for i = 1 : num_samples
    cost = cost + sumsqr(expected_ft(shuffled_indeces(i),:)' - C * sigma * measured_ft(shuffled_indeces(i),:)' + o);
    clc;
    100*i/num_samples
end

cost = cost / num_samples;

lam = 0;

cost = cost + lam * sumsqr(C - eye(6));

opti.minimize(cost);

opti.solver('ipopt');
sol = opti.solve();

if config.scaling_opt
    output_casadi.C = sol.value(C) * sigma;
else
    output_casadi.C = sol.value(C);
end
output_casadi.o = sol.value(o);

end