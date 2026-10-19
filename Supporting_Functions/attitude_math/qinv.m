function [qs] = qinv(q)
% function [qs] = qinv(q)
% quaternion inverse, even for non-normal q
%   qmult(qs,q) = qmult(q,qs) = [0 0 0 1]
% Works for a single 4x1, 1x4, or an Nx4 matrix of quaternions.
%
% Fully vectorized to eliminate branching logic and manual array expansion.

[m, n] = size(q);
is_col = (n == 1);

% Normalize to Nx4 for processing
if is_col
    q = q.';
    [m, n] = size(q);
end

if n ~= 4
    error('[qinv] Shape error: input must be (N,4) or (4,N)');
end

% 1. Get the conjugate (Negate vector part, keep scalar part)
qs = -q;
qs(:, 4) = q(:, 4);

% 2. Calculate the squared magnitude (x = sum of squares of components)
% sum(q.^2, 2) calculates the sum across columns for each row
x = sum(q.^2, 2);

% 3. Divide conjugate by squared magnitude
% Using implicit expansion to divide the Nx4 matrix by the Nx1 magnitudes
% Added protection against division by zero (sets mag to 1 to avoid NaN)
x_safe = x;
x_safe(x == 0) = 1;
qs = qs ./ x_safe;

% Restore original column orientation if necessary
if is_col
    qs = qs.';
end

end