function q = vecab2q(a, b)
% function q = vecab2q(a,b)
% get a quaternion q that rotates vector(s) a to vector(s) b
% Handles N x 3 inputs natively.

[ma, na] = size(a);
[mb, nb] = size(b);

% Standardize to N x 3
if na ~= 3 && ma == 3, a = a.'; [ma, na] = size(a); end
if nb ~= 3 && mb == 3, b = b.'; [mb, nb] = size(b); end

N = max(ma, mb);
if ma == 1 && N > 1, a = repmat(a, N, 1); end
if mb == 1 && N > 1, b = repmat(b, N, 1); end

% 1. Normalize vectors
norm_a = sqrt(sum(a.^2, 2));
norm_b = sqrt(sum(b.^2, 2));

% Handle zero-length vectors by defaulting to [0 0 1]
zero_a = norm_a < eps;
zero_b = norm_b < eps;
a(zero_a, :) = repmat([0 0 1], sum(zero_a), 1);
b(zero_b, :) = repmat([0 0 1], sum(zero_b), 1);
norm_a(zero_a) = 1;
norm_b(zero_b) = 1;

u_a = a ./ norm_a;
u_b = b ./ norm_b;

% 2. Compute cross product and dot product
% Cross product (rotation axis)
v = [u_b(:,2).*u_a(:,3) - u_b(:,3).*u_a(:,2), ...
     u_b(:,3).*u_a(:,1) - u_b(:,1).*u_a(:,3), ...
     u_b(:,1).*u_a(:,2) - u_b(:,2).*u_a(:,1)];

v_mag = sqrt(sum(v.^2, 2));
d = sum(u_a .* u_b, 2); % cos(theta)

% 3. Handle Colinear cases (v_mag near 0)
colinear = (1 == 1 + v_mag);
q = zeros(N, 4);

% --- Path A: Standard (Not colinear) ---
idx_std = ~colinear;
if any(idx_std)
    % Normalize axis and get angle
    axis_std = v(idx_std, :) ./ v_mag(idx_std);
    angle_std = acos(max(-1, min(1, d(idx_std))));
    q(idx_std, :) = vec2q(axis_std, angle_std);
end

% --- Path B: Colinear ---
if any(colinear)
    % Vectors are either same direction (d=1) or opposite (d=-1)
    opp_idx = colinear & (d < 0);
    same_idx = colinear & (d >= 0);
    
    % For "same", return identity
    if any(same_idx)
        q(same_idx, :) = repmat([0 0 0 1], sum(same_idx), 1);
    end
    
    % For "opposite", find any orthogonal axis
    if any(opp_idx)
        % Try crossing with X-axis, if that fails, use Y-axis
        a_opp = u_a(opp_idx, :);
        axis_alt = [zeros(sum(opp_idx),1), a_opp(:,3), -a_opp(:,2)];
        
        % Check for zero-length alt axes (occurs if a was [1 0 0])
        fail = sqrt(sum(axis_alt.^2, 2)) < eps;
        if any(fail)
            axis_alt(fail, :) = [a_opp(fail, 3), zeros(sum(fail),1), -a_opp(fail, 1)];
        end
        
        % Normalize the fallback axis and rotate 180 deg (pi)
        axis_alt = axis_alt ./ sqrt(sum(axis_alt.^2, 2));
        q(opp_idx, :) = vec2q(axis_alt, pi);
    end
end

end