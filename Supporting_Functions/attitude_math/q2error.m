function [verr] = q2error(qdes, qact, mflag)
% function [verr] = q2error( qdes, qact, mflag)
% function [verr] = q2error( qdes, qact)
% quaternion error between two quaternions
%    qerr = qdes.qact* %    or qerr = qmult(qdes, qcon(qact))
% mflag method flag: 
%  mflag = 0 (default)
%      small error approximation
%      verr = 2*qerr(1:3)
%  mflag = 1
%      exact computation using q2vec
%
% input quaternions assumed normalized and properized.
% Fully vectorized leveraging modernized helper functions.

if nargin < 3
  mflag = 0;
end
if nargin < 2
  error('[q2error] needs at least 2 inputs!')
end

flag_col = [0 0];

% ensure rows
nd0 = size(qdes);
[m, n] = size(qdes);
if n ~= 4
  flag_col(1) = 1;
  if m ~= 4
    disp(['size(qdes) ' num2str(nd0)])
    error('[q2error] need quaternion qdes')
  end
  qdes = qdes.';
end

na0 = size(qact);
[ma, na] = size(qact);
if na ~= 4
  flag_col(2) = 1;
  if ma ~= 4
    disp(['size(qact) ' num2str(na0)])
    error('[q2error] need quaternion qact')
  end
  qact = qact.';
end

if flag_col(1) ~= flag_col(2)
  disp(['size(qdes) ' num2str(nd0)])
  disp(['size(qact) ' num2str(na0)])
  error('[q2error] size of arguments do not match (row/column)')
end

% --- VECTORIZED COMPUTATION ---

% Compute error. The modernized qmult and qcon naturally handle N x 4 arrays.
qerr = qmult(qdes, qcon(qact));

if mflag == 0
  % simple linear approx
  verr = 2.0 .* qerr(:, 1:3);
  
  % Logical indexing replaces find() for sign flipping
  neg_idx = qerr(:, 4) < 0;
  verr(neg_idx, :) = -verr(neg_idx, :);
else
  % exact error
  % The modernized qnorm and q2vec natively process arrays.
  [v, theta] = q2vec(qnorm(qerr));
  
  % Implicit expansion automatically broadcasts the N x 1 theta 
  % across the N x 3 vector matrix v.
  verr = v .* theta;
end

% Restore original column orientation if needed
if flag_col(1)
  verr = verr.';
end

end