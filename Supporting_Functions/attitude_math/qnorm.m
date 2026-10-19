function y = qnorm(a)
% function y = qnorm(a)
% Normalize quaternion(s) and enforce a positive scalar (4th) component.
% Works for a single 4x1 or 1x4 quaternion, or an Nx4 matrix of quaternions.
% 
% Fully vectorized to eliminate recursion, find(), and repmat equivalents.

% Check if input is a column vector to restore shape at the end
is_col = (size(a, 2) == 1);
if is_col
    a = a.';
end

% Calculate the magnitude of each quaternion (row-wise 2-norm)
% sum(a.^2, 2) sums across columns for each row directly.
s = sqrt(sum(a.^2, 2));

% Prevent division by zero for zero-length quaternions
s(s == 0) = 1;

% Normalize. Modern MATLAB's implicit expansion automatically handles 
% dividing an Nx4 matrix by an Nx1 vector.
y = a ./ s;

% Enforce positive scalar component (4th element)
% Logical indexing is much faster and cleaner than using find()
neg_idx = y(:, 4) < 0;
y(neg_idx, :) = -y(neg_idx, :);

% Restore original column orientation if needed
if is_col
    y = y.';
end

end