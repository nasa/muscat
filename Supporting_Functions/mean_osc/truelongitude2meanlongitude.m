function meanlongitude = truelongitude2meanlongitude(a, Psi, tq1, tq2)
% truelongitude2meanlongitude Converts true long. Psi to mean long. Lambda
%   meanlongitude = truelongitude2meanlongitude(a, Psi, tq1, tq2) computes
%   the mean longitude from true longitude
%
%   INPUTS:
%   a: semi-major axis
%   Psi: true longitude = theta + Omega = tilde_w+f
%   tq1: equinoctial ecc. vector 1 = e*cos(tilde_w)
%   tq2: equinoctial ecc. vector 2 = e*sin(tilde_w)
%
%   OUTPUTS:
%   meanlongitude: mean longitude Lambda = M + w + Omega

eta = sqrt(1 - tq1 ^ 2 - tq2 ^ 2);
beta = 1 / (eta * (1 + eta));
R = (a * eta ^ 2) / (1 + tq1 * cos(Psi) + tq2 * sin(Psi));

num = R * (1 + beta * tq1 ^ 2) * sin(Psi) - beta * R * tq1 * tq2 * cos(Psi) + a * tq2;
den = R * (1 + beta * tq2 ^ 2) * cos(Psi) - beta * R * tq1 * tq2 * sin(Psi) + a * tq1;

Gamma = atan2(num, den);
Gamma = mod(Gamma, 2 * pi);

Lambda = Gamma - tq1 * sin(Gamma) + tq2 * cos(Gamma);
Lambda = mod(Lambda, 2 * pi);

% strange modulus-related corrections
if (Psi < 0)
    kk_plus = 0;
    quad_plus = 0;
    
    while (Psi < 0)
        kk_plus = kk_plus + 1;
        Psi = Psi + (2 * pi);
    end
    
    if (Psi < (pi / 2)) && (Lambda > (3 * pi / 2))
        quad_plus = 1;
    elseif (Lambda < (pi / 2)) && (Psi > (3 * pi / 2))
        quad_plus = -1;
    end
    
    Lambda = Lambda - (kk_plus + quad_plus) * (2 * pi);
else
    kk_minus = 0;
    quad_minus = 0;
    
    while (Psi >= (2 * pi))
        kk_minus = kk_minus + 1;
        Psi = Psi - (2 * pi);
    end
    
    if (Psi < (pi / 2)) && (Lambda > (3 * pi / 2))
        quad_minus = -1;
    elseif (Lambda < (pi / 2)) && (Psi > (3 * pi / 2))
        quad_minus = 1;
    end
    
    Lambda = Lambda + (kk_minus + quad_minus) * (2 * pi);
end

meanlongitude = Lambda;
end
