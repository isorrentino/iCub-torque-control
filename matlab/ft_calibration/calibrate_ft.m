function [opt.C, opt.o] = calibrate_ft(C_firmware, expected_ft, meas_ft)

opt.C = 0;

% argmin||fexpected - ftrue||^2
% argmin||fexpected - (fmeas + o)||^2
% Ax = b
% 

end

