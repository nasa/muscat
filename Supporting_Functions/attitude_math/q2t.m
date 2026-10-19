function [t] = q2t(q)
% function [t] = q2t(q)
% convert a quaternion (or N quaternions) to a direction cosine matrix.
% Returns a 3x3xN matrix of DCMs.
%
% Fully vectorized to eliminate for-loops and handle multidimensional output.

[m, n] = size(q);

% Normalize input to N x 4 to unify the processing logic
if n ~= 4
    if m == 4
        q = q.';
    else
        error('[q2t] Shape error: input must be an (N,4) or (4,N) matrix.')
    end
end

N = size(q, 1);

% Extract components for vectorized multiplication
q1 = q(:, 1);
q2 = q(:, 2);
q3 = q(:, 3);
q4 = q(:, 4);

% Compute required products
qq41 = q4 .* q1;
qq42 = q4 .* q2;
qq43 = q4 .* q3;

qq12 = q1 .* q2;
qq13 = q1 .* q3;
qq23 = q2 .* q3;

qq11 = q1 .* q1;
qq22 = q2 .* q2;
qq33 = q3 .* q3;
qq44 = q4 .* q4;

den = qq11 + qq22 + qq33 + qq44; % norm^2 of q 

% Identify degenerate quaternions (norm near 0)
zero_idx = abs(den) < 1e-10;

% Prevent division by zero (we will overwrite these specific outputs later)
den(zero_idx) = 1;

sc = 2.0 ./ den;

% Preallocate the 3x3xN output matrix
t = zeros(3, 3, N);

% Populate the DCM using 3rd-dimension assignment
t(1, 1, :) = sc.*(qq44+qq11) - 1.0;
t(1, 2, :) = sc.*(qq12+qq43);
t(1, 3, :) = sc.*(qq13-qq42);

t(2, 1, :) = sc.*(qq12-qq43);
t(2, 2, :) = sc.*(qq44+qq22) - 1.0;
t(2, 3, :) = sc.*(qq23+qq41);

t(3, 1, :) = sc.*(qq13+qq42);
t(3, 2, :) = sc.*(qq23-qq41);
t(3, 3, :) = sc.*(qq44+qq33) - 1.0;

% Overwrite zero-norm quaternions with identity matrices
if any(zero_idx)
    t(1, 1, zero_idx) = 1; t(1, 2, zero_idx) = 0; t(1, 3, zero_idx) = 0;
    t(2, 1, zero_idx) = 0; t(2, 2, zero_idx) = 1; t(2, 3, zero_idx) = 0;
    t(3, 1, zero_idx) = 0; t(3, 2, zero_idx) = 0; t(3, 3, zero_idx) = 1;
end

end