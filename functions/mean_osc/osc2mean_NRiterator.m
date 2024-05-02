function mean_equi_elem = osc2mean_NRiterator(osc_equi_elem, tol)
% osc2mean_NRiterator   NR iteration function for improved osc2mean
%   mean_equi_elem = osc2mean_NRiterator(osc_equi_elem, tol) iteratively
%   computes the mean equinoctial orbital elements from an osculating set,
%   using a NR iteration containing the Jacobian of osc2mean transformation
%
%   INPUTS:
%   osc_equi_elem: osculating equinoctial orbital element vector
%   tol: Iterative solver tolerance (generally 1e-12)
%
%   OUTPUTS:
%   mean_equi_elem: mean equinocital orbital element vector

mean_equi_elem = osc_equi_elem;
R = 1;
niter = 0;

while abs(R) > tol
    niter = niter + 1;
    [~, osc_loop, ~] = transformationmatrix_osc2mean_equinoctial(mean_equi_elem);
    delta = osc_equi_elem - osc_loop;
    R = norm(delta, inf);
    mean_equi_elem = mean_equi_elem + delta;
    
    if niter > 100
        disp('Osc2Mean iterations > 100');
        break;
    end
    
end

end
