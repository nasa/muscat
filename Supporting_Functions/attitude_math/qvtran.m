function [vb] = qvtran(qba, va)
% function [vb] = qvtran(qba,va)
%
% cover for uqvtran, allowing various shape matches
% generally require qba(n,4) va(n,3)
% - allow  q (1,4) or (4,1) to match v
%
% Fully vectorized to eliminate for-loops and internal function calls.

m0 = size(qba);
mv0 = size(va);
flag_col = [0 0];

[m,n] = size(qba);
if n ~= 4 
  flag_col(1) = 1;
  if m ~= 4
    disp([' qba is size ' num2str(m0) ', need (n,4)'])
    error('[qvtran] qba shape error')
  end
  qba = qba.';
  [m,n] = size(qba);
end 

[mv,nv] = size(va);
if flag_col(1)
  % since q is col, assume v is as well
  if mv ~= 3
    if nv ~= 3
      disp([' va  is size ' num2str(mv0) ', need (n,3)' ])
      error('[qvtran] va shape error')
    end
  else
    flag_col(2) = 1;
    va = va.';
    [mv,nv] = size(va);
  end 
else
  % assume v row
  if nv ~= 3
    flag_col(2) = 1;
    if mv ~= 3
      disp([' va  is size ' num2str(mv0) ', need (n,3)' ])
      error('[qvtran] va shape error')
    end
    va = va.';
    [mv,nv] = size(va);
  end 
end

if flag_col(1) ~= flag_col(2)
  disp(['size(qba) ' num2str(m0)])
  disp(['size(va)  ' num2str(mv0)])
  disp(['flag column ' num2str(flag_col)])
  error('[qvtran] size of arguments do not match (row/column)')
end

% at this point expect both to be (n,4) (n,3)
if (m ~= mv) && (m ~= 1) && (mv ~= 1)
  disp(['size(qba) ' num2str(m0)])
  disp(['size(va)  ' num2str(mv0)])
  error('[qvtran] size of arguments do not match')
end

% --- VECTORIZED COMPUTATION ---

% Extract components. MATLAB's implicit expansion will handle cases 
% where one input is 1xN and the other is MxN automatically.
q1 = qba(:, 1);
q2 = qba(:, 2);
q3 = qba(:, 3);
q4 = qba(:, 4);

v1 = va(:, 1);
v2 = va(:, 2);
v3 = va(:, 3);

% Precompute scalar quantities
alpha = 2.0 .* (q1.*v1 + q2.*v2 + q3.*v3);
beta  = -2.0 .* q4;
gamma = 2.0 .* q4.^2 - 1.0;

% Compute the transformed vector components
vb1 = alpha.*q1 + beta.*(q2.*v3 - q3.*v2) + gamma.*v1;
vb2 = alpha.*q2 + beta.*(q3.*v1 - q1.*v3) + gamma.*v2;
vb3 = alpha.*q3 + beta.*(q1.*v2 - q2.*v1) + gamma.*v3;

% Concatenate into final (n,3) matrix
vb = [vb1, vb2, vb3];

% Restore original column orientation if requested
if flag_col(1)
  vb = vb.';
end

end