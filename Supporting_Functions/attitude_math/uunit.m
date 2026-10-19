function [mag, u] = uunit(v)
% function [mag, u] = uunit(v)
% function [u] = uunit(v)
% Unitize sets of 3-vectors, handles rows (N,3) or columns (3,N).
% If only one output is requested, it returns the unitized vectors.

[m, n] = size(v);

if n == 3
    % Case: Set of rows (N, 3)
    mag = sqrt(sum(v.^2, 2));
    
    % Avoid division by zero
    mag_safe = mag;
    mag_safe(mag == 0) = 1;
    
    % Implicit expansion handles (N,3) ./ (N,1)
    u_out = v ./ mag_safe;
    
elseif m == 3
    % Case: Set of columns (3, N)
    mag = sqrt(sum(v.^2, 1));
    
    % Avoid division by zero
    mag_safe = mag;
    mag_safe(mag == 0) = 1;
    
    % Implicit expansion handles (3,N) ./ (1,N)
    u_out = v ./ mag_safe;
    
else
    error('[uunit] Only works for sets of 3-vectors (N,3 or 3,N).');
end

% Handle output arguments
if nargout < 2
    % If user called: u = uunit(v)
    mag = u_out;
else
    % If user called: [mag, u] = uunit(v)
    u = u_out;
end

end