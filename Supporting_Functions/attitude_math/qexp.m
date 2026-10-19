function [y, vu, theta] = qexp(q)
% function [y,vu,theta] = qexp(q)
% Get the exponential of a quaternion (or a matrix of quaternions)
%
% Fully vectorized to eliminate find() calls and manual array replication.

[m, n] = size(q);
is_transposed = false;

% Normalize to N x 4
if n ~= 4
    if m == 4
        q = q.';
        [m, n] = size(q);
        is_transposed = true;
    else
        error('[qexp] Input must be (N,4) or (4,N)');
    end
end

% 1. Extract vector part (1:3) and scalar part (4)
v = q(:, 1:3);
s = q(:, 4);

% 2. Calculate magnitude of the vector part (rotation angle)
theta = sqrt(sum(v.^2, 2));

% 3. Calculate unit vector (vu)
% We use logical indexing to handle small angles where theta ~ 0
is_small = (1 == 1 + theta);
not_small = ~is_small;

vu = zeros(m, 3);
% Default for small angles is a generic unit vector [0, 0, 1]
vu(is_small, 3) = 1.0;

% For standard angles, normalize the vector part
if any(not_small)
    % Implicit expansion: divide each row of v by its corresponding theta
    vu(not_small, :) = v(not_small, :) ./ theta(not_small);
end

% 4. Compute Exponential components
% qexp(s + v) = exp(s) * [vu * sin(theta), cos(theta)]
es = exp(s);
sn = sin(theta);
cn = cos(theta);

% Implicit expansion: multiply components by exp(s) and sin(theta)
y_vec = es .* vu .* sn;
y_scalar = es .* cn;

% Final assembly
y = [y_vec, y_scalar];

% Restore original orientation if necessary
if is_transposed
    y = y.';
    vu = vu.';
    theta = theta.';
end

end