function m = tilde(v)
% function m = tilde(v)
% Build cross product skew-symmetric matrices for vector(s) v.
% If v is (N,3), returns (3,3,N).
% If v is (3,1), returns (3,3).
%
% cross(v, u) == tilde(v) * u

[rows, cols] = size(v);

% 1. Standardize input to N x 3
if cols ~= 3
    if rows == 3
        v = v.';
        [rows, ~] = size(v);
    else
        error('[tilde] Input must be a 3-element vector or an (N,3) matrix.');
    end
end

N = rows;

% 2. Preallocate the 3x3xN array
m = zeros(3, 3, N);

% 3. Extract components
v1 = v(:, 1);
v2 = v(:, 2);
v3 = v(:, 3);

% 4. Fill the skew-symmetric matrix elements for all N vectors at once
% Plane 1: Row 1
m(1, 2, :) = -v3;
m(1, 3, :) =  v2;

% Plane 2: Row 2
m(2, 1, :) =  v3;
m(2, 3, :) = -v1;

% Plane 3: Row 3
m(3, 1, :) = -v2;
m(3, 2, :) =  v1;

% If input was just a single vector, squeeze the result to 3x3
if N == 1
    m = reshape(m, [3, 3]);
end

end