function [q] = t2q(t)
% function [q] = t2q(t)
% Convert Direction Cosine Matrix (3x3xN) to Quaternions (Nx4)
% Vectorized Shepperd's Algorithm for numerical stability.

[r, c, N] = size(t);
if r ~= 3 || c ~= 3
    error('[t2q] Input must be a 3x3xN matrix.');
end

% Extract diagonal elements for all N matrices
t11 = squeeze(t(1,1,:));
t22 = squeeze(t(2,2,:));
t33 = squeeze(t(3,3,:));

% Trace
tr = t11 + t22 + t33;

% We need to find which of [tr, t11, t22, t33] is the largest for each matrix
% to determine the stable numerical path.
vals = [tr, t11, t22, t33];
[~, i] = max(vals, [], 2); 
% i=1: Trace is largest (q4 path)
% i=2: t11 is largest   (q1 path)
% i=3: t22 is largest   (q2 path)
% i=4: t33 is largest   (q3 path)

% Preallocate q
q = zeros(N, 4);

% Squeeze off-diagonal elements for vectorization
t12 = squeeze(t(1,2,:)); t13 = squeeze(t(1,3,:));
t21 = squeeze(t(2,1,:)); t23 = squeeze(t(2,3,:));
t31 = squeeze(t(3,1,:)); t32 = squeeze(t(3,2,:));

%% Case 1: Trace is largest (i=1)
m1 = (i == 1);
if any(m1)
    s = sqrt(tr(m1) + 1.0) * 2; 
    q(m1,4) = 0.25 * s;
    q(m1,1) = (t23(m1) - t32(m1)) ./ s;
    q(m1,2) = (t31(m1) - t13(m1)) ./ s;
    q(m1,3) = (t12(m1) - t21(m1)) ./ s;
end

%% Case 2: t11 is largest (i=2)
m2 = (i == 2);
if any(m2)
    s = sqrt(1.0 + t11(m2) - t22(m2) - t33(m2)) * 2;
    q(m2,4) = (t23(m2) - t32(m2)) ./ s;
    q(m2,1) = 0.25 * s;
    q(m2,2) = (t12(m2) + t21(m2)) ./ s;
    q(m2,3) = (t13(m2) + t31(m2)) ./ s;
end

%% Case 3: t22 is largest (i=3)
m3 = (i == 3);
if any(m3)
    s = sqrt(1.0 + t22(m3) - t11(m3) - t33(m3)) * 2;
    q(m3,4) = (t31(m3) - t13(m3)) ./ s;
    q(m3,1) = (t12(m3) + t21(m3)) ./ s;
    q(m3,2) = 0.25 * s;
    q(m3,3) = (t23(m3) + t32(m3)) ./ s;
end

%% Case 4: t33 is largest (i=4)
m4 = (i == 4);
if any(m4)
    s = sqrt(1.0 + t33(m4) - t11(m4) - t22(m4)) * 2;
    q(m4,4) = (t12(m4) - t21(m4)) ./ s;
    q(m4,1) = (t13(m4) + t31(m4)) ./ s;
    q(m4,2) = (t23(m4) + t32(m4)) ./ s;
    q(m4,3) = 0.25 * s;
end

% Standardize: ensure q4 is positive
q = qnorm(q);

end