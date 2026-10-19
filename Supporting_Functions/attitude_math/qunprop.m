function qu = qunprop(q)
% function qu = qunprop(q)
% un-properize q for interpolation
% assumes a set of rows of q's (N x 4)
% assumes small rotations (< pi) expected between successive rows
%
% Fully vectorized using cumulative products to eliminate the for-loop.

[m, n] = size(q);
if n ~= 4
    if m == 4
        q = q.';
    else
        error('[qunprop] Input must be (N,4) or (4,N)');
    end
end

% 1. Calculate the difference between successive quaternions
% This uses your modernized qdiff which handles the matrix natively.
qd = qdiff(q);

% 2. Identify where the rotation jump occurs.
% In properized quaternions, a "jump" results in a negative scalar (4th) component.
% We create a vector of signs: 1 if smooth, -1 if a flip is detected.
jump_detected = ones(size(q, 1), 1);
jump_detected(2:end) = sign(qd(:, 4));

% 3. Calculate the cumulative sign.
% The logic 'sc = -sc' is equivalent to a cumulative product of the signs.
% If row 10 has flipped once, sc is -1. If it flips again at row 20, sc becomes 1.
sc_vector = cumprod(jump_detected);

% 4. Apply the scalar vector to the entire quaternion matrix.
% Implicit expansion multiplies each row i by its corresponding sc_vector(i).
qu = q .* sc_vector;

% Restore original orientation if necessary
if n ~= 4
    qu = qu.';
end

end