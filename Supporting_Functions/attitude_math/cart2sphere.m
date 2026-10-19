function [ang] = cart2sphere(vs)
% function [ang]=cart2sphere(vs)
% transform vector to spherical coord
% assume vs is n x 3
% works also for vs 3 x 1
%   angles = [theta,phi]
%
% Fully vectorized to eliminate for-loops and conditionals.

[n, m] = size(vs);
if m == 1
  vs = vs.'; % Handle 3x1 case with non-conjugate transpose
end

% Extract components
x = vs(:, 1);
y = vs(:, 2);
z = vs(:, 3);

% Calculate theta (azimuthal angle)
% MATLAB's atan2 safely handles all quadrants, including x=0 and y=0
theta = atan2(y, x);

% Calculate phi (polar/inclination angle)
% The original code computed rho = x/cos(theta) or y/sin(theta).
% Using hypot(x, y) computes sqrt(x^2 + y^2) directly, avoiding all 
% divide-by-zero singularity checks.
rho = hypot(x, y); 
phi = atan2(rho, z);

% Concatenate into final n x 2 matrix
ang = [theta, phi];

end