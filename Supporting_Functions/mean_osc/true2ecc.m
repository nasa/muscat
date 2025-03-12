function E = true2ecc(f, e)
% true2ecc Computes eccentric anomaly given true anomaly and eccentricity
%
% Inputs:
%      f - mean anomaly [rad]
%      e - eccentricity of orbit
%
% Outputs:
%      E - eccentric anomaly [rad]

f = mod(f, 2 * pi);

E = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(f / 2));

% Make sure E sits in the correct semi-plane
E = wrapToPi(E);

end
