function v = sphere2cart(ang)
% function v = sphere2cart(ang)
% Transform spherical coordinates to a Cartesian unit vector.
% ang is [theta, phi] - handles (N,2) or (2,N).
% v is returned as (N,3).
%
% Optimized version with robust dimension handling.

[m, n] = size(ang);
is_transposed = false;

% Normalize input to N x 2
if n ~= 2
    if m == 2
        ang = ang.';
        [m, n] = size(ang);
        is_transposed = true;
    else
        error('[sphere2cart] Shape error: input must be (N,2) or (2,N)');
    end
end

% Extract angles for clarity and speed
theta = ang(:, 1);
phi   = ang(:, 2);

% Precompute sin(phi) as it is used twice
sin_phi = sin(phi);

% Generate the Cartesian components
% x = cos(theta)*sin(phi)
% y = sin(theta)*sin(phi)
% z = cos(phi)
v = [cos(theta).*sin_phi, sin(theta).*sin_phi, cos(phi)];

% Restore original orientation if input was (2,N)
if is_transposed
    v = v.';
end

end