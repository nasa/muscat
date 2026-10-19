function [q] = qatt(p, s)
% function [q] = qatt(p,s)
% form quaternion based on primary vector p(3), secondary vector s(3) 
% p, s can be rows or columns (matching) q returned matches
% p, s may be single vectors, e.g. p(1,3) s(n,3) => q(n,4)
%
% Fully vectorized to eliminate for-loops, conditional branching, 
% and the external uqatt function.

np0 = size(p);
ns0 = size(s);
[np, mp] = size(p);
[ns, ms] = size(s);

% allow scalar p or s to be column or row
% turn into rows
flag_col = [0 0];
if mp ~= 3
  % can't be row
  flag_col(1) = 1;
  if np ~= 3
    error(['[qatt] need vector p, have size ' num2str(np0)])
  end
  p = p.';
  [np, ~] = size(p);
end

if ms ~= 3
  % can't be row
  flag_col(2) = 1;
  if ns ~= 3
    error(['[qatt] need vector s, have size ' num2str(ns0)])
  end
  s = s.';
  [ns, ~] = size(s);
end

if flag_col(1) ~= flag_col(2)
  disp(['size(p) ' num2str(np0)])
  disp(['size(s) ' num2str(ns0)])
  error('[qatt] size of arguments do not match (row/column)')
end

% at this point expect both to be (n,3)
if (np ~= ns) && (np ~= 1) && (ns ~= 1)
  disp(['size(p) ' num2str(np0)])
  disp(['size(s) ' num2str(ns0)])
  error('[qatt] size of arguments do not match')
end

N = max(np, ns);

% --- VECTORIZED COMPUTATION ---

% Expand singletons to N x 3 so logical indexing masks match dimensions safely
if np == 1 && N > 1
    p = repmat(p, N, 1);
end
if ns == 1 && N > 1
    s = repmat(s, N, 1);
end

epsilon = 1e-4;

p1 = p(:,1); p2 = p(:,2); p3 = p(:,3);
s1 = s(:,1); s2 = s(:,2); s3 = s(:,3);

% Compute negative dot product, saturated to [-1, 1] (replaces usat)
d = -(p1.*s1 + p2.*s2 + p3.*s3);
d = usat(d, 1.0);

% Compute sines and cosines of angles
z = sqrt(1.0 - d.^2);
x = sqrt(p1.^2 + p2.^2);

% Preallocate intermediate variables
a = zeros(N, 1);       b = ones(N, 1);
e_val = zeros(N, 1);   f_val = zeros(N, 1);
st = zeros(N, 1);      ct = ones(N, 1);
e_final = zeros(N, 1); f_final = zeros(N, 1);

% --- Condition 1: Polar pointing check (x < epsilon) ---
mask_x = x < epsilon;
not_x = ~mask_x;

% For x < epsilon:
e_val(mask_x) = s2(mask_x);
f_val(mask_x) = s1(mask_x) .* p3(mask_x);
% a is already 0, b is already 1

% For x >= epsilon:
% Prevent division by zero warnings by forcing x=1 where mask_x is true
x_safe = x; 
x_safe(mask_x) = 1.0; 

e_val(not_x) = (s2(not_x).*p1(not_x) - s1(not_x).*p2(not_x)) ./ x_safe(not_x);
f_val(not_x) = -(d(not_x).*p3(not_x) + s3(not_x)) ./ x_safe(not_x);

mask_p1 = p1 > 0;
idx_p1_pos = not_x & mask_p1;
idx_p1_neg = not_x & ~mask_p1;

a(idx_p1_pos) = p2(idx_p1_pos);
b(idx_p1_pos) = x(idx_p1_pos) + p1(idx_p1_pos);

a(idx_p1_neg) = x(idx_p1_neg) - p1(idx_p1_neg);
b(idx_p1_neg) = p2(idx_p1_neg);


% --- Condition 2: Undetermined problem check (z < epsilon) ---
mask_z = z < epsilon;
not_z = ~mask_z;
% For z < epsilon: st is already 0, ct is already 1

% For z >= epsilon:
mask_f = f_val > 0;
idx_f_pos = not_z & mask_f;
idx_f_neg = not_z & ~mask_f;

st(idx_f_pos) = e_val(idx_f_pos);
ct(idx_f_pos) = z(idx_f_pos) + f_val(idx_f_pos);

st(idx_f_neg) = z(idx_f_neg) - f_val(idx_f_neg);
ct(idx_f_neg) = e_val(idx_f_neg);


% --- Condition 3: Final co-declination mapping ---
mask_p3 = p3 > 0;

e_final(mask_p3) = x(mask_p3);
f_final(mask_p3) = 1.0 + p3(mask_p3);

e_final(~mask_p3) = 1.0 - p3(~mask_p3);
f_final(~mask_p3) = x(~mask_p3);


% --- Final Quaternion Construction ---
q1 = e_final .* (st .* b - ct .* a);
q3 = f_final .* (ct .* a + st .* b);
q2 = e_final .* (ct .* b + st .* a);
q4 = f_final .* (ct .* b - st .* a);

% Combine and normalize
q = [q1, q2, q3, q4];

% Normalizing assumes qnorm has been modernized to handle N x 4 arrays.
q = qnorm(q);

% Restore original column orientation if requested
if flag_col(1)
  q = q.';
end

end