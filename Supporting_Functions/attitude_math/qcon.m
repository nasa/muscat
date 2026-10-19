function qs = qcon(q)
% function qs = qcon(q)
% Compute the conjugate of a quaternion.
% Handles a single quaternion, an N x 4 array, or a 4 x N array.
%
% Fully vectorized to eliminate nested conditionals and unify processing.

[m, n] = size(q);
is_transposed = false;

% Normalize input to N x 4 to unify the processing logic
if n ~= 4
    if m == 4
        q = q.';
        is_transposed = true;
    else
        error('[qcon] Shape error: input must be an (N,4) or (4,N) matrix.')
    end
end

% --- VECTORIZED COMPUTATION ---

% Conjugate rule: Negate the vector components (1-3), keep the scalar (4)
qs = -q;
qs(:, 4) = q(:, 4);

% Restore the original 4 x N orientation if it was transposed
if is_transposed
    qs = qs.';
end

end