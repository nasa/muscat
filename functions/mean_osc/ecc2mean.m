function M = ecc2mean(E, e)
% ecc2mean Computes mean anomaly given eccentric anomaly and eccentricity
%
% Inputs:
%     E - eccentric anomaly [rad]
%     e - eccentricity of orbit
%
% Outputs:
%     M - mean anomaly [rad]

E = mod(E, 2 * pi);

% Kepler's equation
M = E - e * sin(E);

end
