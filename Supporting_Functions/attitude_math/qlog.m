function y = qlog(q)
% function y = qlog(q)
% Get the log of a quaternion
%   q == qexp(qlog(q))
% Works for a single quaternion or an Nx4 matrix of rows.
%
% Fully vectorized to eliminate branching and manual array concatenation.

[m, n] = size(q);
is_transposed = false;

% Normalize to N x 4
if n ~= 4
    if m == 4
        q = q.';
        [m, n] = size(q);
        is_transposed = true;
    else
        error('[qlog] Input must be (N,4) or (4,N)');
    end
end

% 1. Calculate the magnitude of the quaternion
mag = sqrt(sum(q.^2, 2));

% 2. Natural log of the magnitude (the scalar part of the log)
% Protection against log(0)
mag_safe = mag;
mag_safe(mag == 0) = 1; 
s = log(mag_safe);

% 3. Normalize the quaternion
% Implicit expansion divides Nx4 matrix by Nx1 magnitude
qu = q ./ mag_safe;

% 4. Extract axis (v) and angle (theta) from the unit quaternion
% This uses your modernized q2vec.m which is already vectorized.
[v, theta] = q2vec(qu);

% 5. Assemble the log: y = [ (v * theta / 2), log(mag) ]
% Implicit expansion multiplies Nx3 vector part by Nx1 theta/2
y_vec = v .* (theta / 2.0);

y = [y_vec, s];

% Restore original orientation if necessary
if is_transposed
    y = y.';
end

end