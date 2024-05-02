function koe = equioe2koe(equioe)
% equioe2koe   Transformation of equinoctial orbital elements to Keplerian
% orbital elements
%   koe = equioe2koe(equioe) computes the nonlinear transformation from
%   equioe to koe
%
%   INPUTS:
%   equioe: Equinoctial orbital element vector [a;Psi;tq1;tq2;p1;p2] in rad
%
%   OUTPUTS:
%   koe: Keplerian orbital element vector [a;e;i;O;w;M] in rad

a = equioe(1);
Psi = equioe(2);
tq1 = equioe(3);
tq2 = equioe(4);
p1 = equioe(5);
p2 = equioe(6);

Omega = atan2(p2, p1);
% i = 2*atan2(p1,cos(Omega));
i = 2 * atan(p1 / cos(Omega));

wtilde = atan2(real(tq2), real(tq1));
e = sqrt(tq1 ^ 2 + tq2 ^ 2);

w = wtilde - Omega;
f = Psi - wtilde;

E = atan2(real(sin(f) * sqrt(1 - e ^ 2)), real(cos(f) + e));
M = E - e * sin(E);

w = mod(w, 2 * pi);
M = mod(real(M), 2 * pi);

if (abs(M - 2 * pi) < eps)
    M = 0;
end

koe = [a; e; i; Omega; w; M];

end
