function y = qdiff(q)
% function y = qdiff(q)
% get diff for a set of q's, stored as rows
% this is like matlab's diff, but for quaternion representations of
% frames, so
%    dq = q(k) . q(k-1)*
% or 
%    dq = qmult( q(k), qcon( q(k-1) ) )
%
% Fully modernized and streamlined.

[m, n] = size(q);
is_transposed = false;

% Normalize input to N x 4 to unify the processing logic
if n ~= 4
    if m == 4
        q = q.';
        [m, ~] = size(q);
        is_transposed = true;
    else
        error('[qdiff] Shape error: input must be an (N,4) or (4,N) matrix.')
    end
end

if m < 2
    % Return identity quaternion if there are not enough rows to difference
    y = [0, 0, 0, 1];
else
    % Compute the difference using native MATLAB array slicing.
    % q(2:end, :) represents q(k)
    % q(1:end-1, :) represents q(k-1)
    y = qmult(q(2:end, :), qcon(q(1:end-1, :)));
end

% Restore original orientation if the input was 4 x N
if is_transposed
    y = y.';
end

end