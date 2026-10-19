function [ra, dec] = v2rade(v)
% function [ra,dec] = v2rade(v)
% Get the Right Ascension and Declination components of vector(s).
% Handles (N,3) or (3,1) inputs.
%
% Fully vectorized to eliminate for-loops and singularity branching.

[m, n] = size(v);

% Handle (1,3) or (3,1) single vector cases
if (m == 1 || n == 1)
    if length(v) ~= 3
        error('[v2rade] Input vector must have 3 elements.');
    end
    v = v(:).'; % Force to 1x3 row
else
    if n ~= 3
        error('[v2rade] Input matrix must be (N,3).');
    end
end

% 1. Unitize the vectors (Calculate magnitude of each row)
mag = sqrt(sum(v.^2, 2));

% Prevent division by zero for zero-length vectors
mag_safe = mag;
mag_safe(mag == 0) = 1;
u = v ./ mag_safe;

% 2. Calculate projection in the xy-plane
v12 = hypot(u(:,1), u(:,2));

% 3. Compute Angles
% ra = atan2(y, x). MATLAB handles the x=0, y=0 case naturally.
ra = atan2(u(:,2), u(:,1));

% dec = atan2(z, projection_in_xy)
dec = atan2(u(:,3), v12);

% 4. Handle Singularity (Polar Pointing)
% If a vector points exactly at the pole, RA is mathematically undefined.
% We follow the legacy convention and force RA to 0 where v12 is near 0.
epsilon = 1e-10;
is_polar = v12 < epsilon;
ra(is_polar) = 0.0;

end