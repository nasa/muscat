function [q] = vec2q(v, theta)
% function [q] = vec2q(v,theta);
% Vectorized axis-angle to quaternion conversion.
% Handles scalar v with array theta, scalar theta with array v, or matched arrays.

nv0 = size(v);
[mv, nv] = size(v);

% Ensure theta is a column vector for implicit expansion
theta = theta(:);
nt = length(theta);

% Handle column vs row inputs for v
flag_col = 0;
if 3 ~= nv
    flag_col = 1;
    if 3 ~= mv
        disp(['size(v) ' num2str(nv0)])
        error('[vec2q] error in v size; need vector (n,3)')
    end
    v = v.';
    [mv, nv] = size(v);
end

% Dimension matching check
if (mv ~= nt) && (mv ~= 1) && (nt ~= 1)
    disp(['size(v) ' num2str(nv0)])
    disp(['length(theta) ' num2str(nt)])
    error('[vec2q] size mismatch') 
end

n = max(mv, nt);

% --- VECTORIZED COMPUTATION ---

% Calculate magnitude of rotation vector(s)
mag = sqrt(sum(v.^2, 2));

% Identify zero-vector cases to avoid division by zero
% Using the 1 == 1 + mag check from original code logic
is_zero = (1 == 1 + mag);

% Prepare alpha and s
% Note: If v is (1,3) and theta is (N,1), MATLAB expands v to match.
% If v is (N,3) and theta is (1,1), MATLAB expands theta to match.
alfa = (mag .* theta) / 2.0;

% Calculate sine-based scaling factor
% We handle the zero-mag case by temporarily setting mag to 1 to avoid NaN,
% then zeroing out the result with a mask.
mag_safe = mag;
mag_safe(is_zero) = 1.0;
s = sin(alfa) ./ mag_safe;
s(is_zero) = 0.0;

% Build quaternion components
% Implicit expansion handles (N,1) * (N,3) or (N,1) * (1,3)
q123 = s .* v;
q4 = cos(alfa);

% Final assembly
% If q4 resulted from a (1,1) theta and (N,3) v, it might be a scalar; 
% we force it to match the number of rows N.
if length(q4) == 1 && n > 1
    q4 = repmat(q4, n, 1);
end

q = [q123, q4];

% Warning for zero vectors
if any(is_zero)
    disp('[vec2q] warning - pivot vector v is zero');
end

% Match input orientation
if flag_col == 1
    q = q.';
end

end