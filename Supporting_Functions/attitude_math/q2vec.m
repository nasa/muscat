function [v, theta] = q2vec(q)
% function [v,theta] = q2vec(q)
% get unit vector and angle 
% works for a single row/column, or an N x 4 set of rows
%
% Fully vectorized to eliminate conditional branching and helper functions.

[m, n] = size(q);

% Convert a 4x1 column vector to a 1x4 row vector to unify processing
if m == 4 && n == 1
    q = q.';
    [m, n] = size(q);
end

% Shape error checking
if n ~= 4
    disp([m, n])
    disp('[q2vec] shape error')
    v = 0;
    theta = 0;
    return;
end

% Normalize the quaternion(s). 
% This assumes your modernized qnorm() from the previous step is on the path.
q = qnorm(q);

% Extract the vector portion (first three columns)
v = q(:, 1:3);

% Calculate the magnitude of the rotation vector (corresponds to sin(theta/2))
% sum(v.^2, 2) sums horizontally across the rows.
sth2 = sqrt(sum(v.^2, 2));

% Prevent division by zero for zero-rotation quaternions (where sth2 == 0)
mag = sth2;
mag(mag == 0) = 1;

% Normalize the rotation vector(s) via implicit expansion
v = v ./ mag;

% Extract the scalar portion (corresponds to cos(theta/2))
cth2 = q(:, 4);

% Get the rotation amount.
% MATLAB's built-in atan2 naturally handles the x=0, y=0 cases and quadrants,
% completely eliminating the need for the external uatanpi function.
theta = 2.0 .* atan2(sth2, cth2);

end