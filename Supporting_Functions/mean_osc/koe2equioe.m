function equioe = koe2equioe(koe)
% koe2equioe   Transformation of Keplerian orbital elements to equinoctial
% orbital elements
%   equioe = koe2equioe(koe) computes the nonlinear transformation from
%   koe to equioe
%
%   INPUTS:
%   koe: Keplerian orbital element vector [a;e;i;O;w;M] in rad
%
%   OUTPUTS:
%   equioe: Equinoctial orbital element vector [a;Psi;tq1;tq2;p1;p2] in rad

a = koe(1);
e = koe(2);
i = koe(3);
Omega = koe(4);
w = koe(5);
M = koe(6);

f = mean2true(M, e);
w_tilde = Omega + w;
Psi = w_tilde + f;
tq1 = e * cos(w_tilde);
tq2 = e * sin(w_tilde);
p1 = tan(i / 2) * cos(Omega);
p2 = tan(i / 2) * sin(Omega);

Psi = mod(Psi, 2 * pi);

if Psi > pi
    Psi = Psi - 2 * pi;
end

equioe = [a; Psi; tq1; tq2; p1; p2];

end
