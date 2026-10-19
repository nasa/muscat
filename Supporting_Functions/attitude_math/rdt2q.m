function [q] = rdt2q(ra, dec, tw)
% function [q] = rdt2q(ra, dec, tw)
% function [q] = rdt2q(ra_dec_tw)
% Change RA, Dec, Twist Euler parameters to a quaternion (Nx4).
%
% Euler angle rotation sequence:
% 1) Rotate Right Ascension about z-axis.
% 2) Rotate ( PI/2 - Declination ) about y-axis.
% 3) Rotate Twist about z-axis.

% --- Handle Input Parsing ---
if nargin == 3
    % Case: Three separate vectors
    ra = ra(:); dec = dec(:); tw = tw(:);
    n = length(ra);
    if length(dec) ~= n || length(tw) ~= n
        error('[rdt2q] All inputs must be the same length');
    end
    flag_tr = 0;
elseif nargin == 1
    % Case: Single matrix (N,3) or (3,N)
    [rows, cols] = size(ra);
    if cols == 3
        % Already (N,3)
        tw = ra(:,3);
        dec = ra(:,2);
        ra_val = ra(:,1); % Rename to avoid conflict with function input name
        ra = ra_val;
        flag_tr = 0;
    elseif rows == 3
        % (3,N) input, transpose to process
        ra_mat = ra.';
        tw = ra_mat(:,3);
        dec = ra_mat(:,2);
        ra = ra_mat(:,1);
        flag_tr = 1;
    else
        error('[rdt2q] Input dimension must be (N,3) or (3,N)');
    end
else
    error('[rdt2q] Requires 1 or 3 input arguments');
end

% --- Vectorized Computation ---
% codec = pi/2 - declination
half_codec = (pi / 2.0 - dec) ./ 2.0;
half_diff   = (tw - ra) ./ 2.0;
half_sum    = (tw + ra) ./ 2.0;

% Precompute sines and cosines to avoid redundant calls
s_codec = sin(half_codec);
c_codec = cos(half_codec);

% Compute components based on the Z-Y-Z Euler logic
q1 = s_codec .* sin(half_diff);
q2 = s_codec .* cos(half_diff);
q3 = c_codec .* sin(half_sum);
q4 = c_codec .* cos(half_sum);

% Assemble the Nx4 matrix
q = [q1, q2, q3, q4];

% Match original orientation if (3,N) was provided
if flag_tr
    q = q.';
end

end