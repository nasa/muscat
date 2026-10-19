function c = qmult(a, b)
% function c = qmult(a,b)
% multiplies 2 quaternions, or 2 sets, stored as rows
% either may be a scalar
%
% Fully vectorized to eliminate for-loops and internal function calls.

na0 = size(a);
nb0 = size(b);
[na, ma] = size(a);
[nb, mb] = size(b);

% allow scalar a or b to be column or row
% turn into rows
flag_col = [0 0];
if ma ~= 4
  % can't be row
  flag_col(1) = 1;
  if na ~= 4
    error(['[qmult] need quaternion a, have size ' num2str(na0)])
  end
  a = a.';
  [na, ma] = size(a);
end

if mb ~= 4
  % can't be row
  flag_col(2) = 1;
  if nb ~= 4
    error(['[qmult] need quaternion b, have size ' num2str(nb0)])
  end
  b = b.';
  [nb, mb] = size(b);
end

if flag_col(1) ~= flag_col(2)
  disp(['size(a) ' num2str(na0)])
  disp(['size(b) ' num2str(nb0)])
  error('[qmult] size of arguments do not match (row/column)')
end

% at this point expect both to be (n,4)
if (na ~= nb) && (na ~= 1) && (nb ~= 1)
  disp(['size(a) ' num2str(na0)])
  disp(['size(b) ' num2str(nb0)])
  error('[qmult] size of arguments do not match')
end

% --- VECTORIZED COMPUTATION ---

% Extract columns. MATLAB's implicit expansion handles cases where 
% one input is 1x4 and the other is Nx4 automatically.
a1 = a(:, 1); a2 = a(:, 2); a3 = a(:, 3); a4 = a(:, 4);
b1 = b(:, 1); b2 = b(:, 2); b3 = b(:, 3); b4 = b(:, 4);

% Perform quaternion multiplication rules across all rows simultaneously
c1 =  a4.*b1 + a3.*b2 - a2.*b3 + a1.*b4;
c2 = -a3.*b1 + a4.*b2 + a1.*b3 + a2.*b4;
c3 =  a2.*b1 - a1.*b2 + a4.*b3 + a3.*b4;
c4 = -a1.*b1 - a2.*b2 - a3.*b3 + a4.*b4;

% Concatenate the results into the final Nx4 array
c = [c1, c2, c3, c4];

% Restore column orientation if the original inputs were columns
if flag_col(1)
  c = c.';
end

end