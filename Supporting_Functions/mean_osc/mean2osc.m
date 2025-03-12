% mean2osc
%   Conversion from mean to osculating orbital elements
%   osc_elem = mean2osc(mean_elem,J2_flag) computes J2-perturbed
%   osculating orbital elements from a set of mean orbital elements.
%
%   INPUTS:
%   mean_elem = [a;e;i;O;w;M]: mean Keplerian orbital element vector
%
%   J2_flag: Flags whether J2 should be considered.
%       J2_flag = 1 -> J2 is enabled, calculate according to algorithm
%       J2_flag = 0 -> J2 is disabled, osc elements = mean elements
%       DEFAULT J2_flag = 1
%
%   OUTPUTS:
%   osc_elem: osculating Keplerian orbital element vector

function osc_elem = mean2osc(mean_elem, J2_flag)
% alpha_osc 6 x 1 or n x 6

% Check inputs
if (nargin < 2) || isempty(J2_flag)
    J2_flag = 1;
end

if (nargin < 1) || isempty(mean_elem)
    error('Must input mean elements set');
end

if size(mean_elem, 2) == 1 && size(mean_elem, 1) == 6
    osc_elem = mean2osc_(mean_elem, J2_flag);
else
    n = size(mean_elem,1);
    osc_elem = zeros(n,6);
    for i=1:n
        osc_elem(i,:) = mean2osc_(mean_elem(i,:), J2_flag);
    end
end
end


function osc_elem = mean2osc_(mean_elem, J2_flag)

% mean_elem(1) = mean_elem(1) * 1e3; % a [km] -> a [m]

% Check inputs
if (nargin < 2) || isempty(J2_flag)
    J2_flag = 1;
end

if (nargin < 1) || isempty(mean_elem)
    error('Must input mean elements set');
end

% Format input to column vector and set tolerance
mean_elem = mean_elem(:);

% With J2, run method
if J2_flag == 1
    
    % Convert to mean equinoctial elements
    mean_equi_elem = koe2equioe(mean_elem);
    
    % Convert to osculating equinoctial elements
    [~, osc_equi_elem] = mean_osc_closed_equi(mean_equi_elem, J2_flag);
    
    % Convert to keplerian elements
    osc_elem = equioe2koe(osc_equi_elem)';
    
    % Format output
    osc_elem = osc_elem(:);
    
    % Without J2, elements are equal
else
    osc_elem = mean_elem;
end

osc_elem(3:6) = wrapToPi(osc_elem(3:6));
% osc_elem(1) = osc_elem(1) / 1e3; % a [m] -> a [km]
end
