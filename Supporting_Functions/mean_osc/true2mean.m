function M = true2mean(f, e)
% true2mean solves Kepler's equation for mean anomaly
%
% Inputs:
%     f - true anomaly [rad]
%     e - eccentricity of orbit
%
% Outputs:
%     M - mean anomaly [rad]

N = floor(f / (2*pi));
E = true2ecc(f - N*2*pi, e);
M = ecc2mean(E, e) + N*2*pi;

end
