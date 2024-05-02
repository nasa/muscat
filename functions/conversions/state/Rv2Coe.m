function coe = Rv2Coe(rv, mu)
coe = VecFcn(@Rv2Coe_, rv, mu);
end

function coe = Rv2Coe_(rv, mu)

% Initialize Cartesian Polistion and Velocity
r = rv(1:3);
v = rv(4:6);

h = cross(r, v);  % Angular momentum vector
W = h/norm(h);    % Unit vector along angular momentum vector

i     = atan2(sqrt(W(1)*W(1) + W(2)*W(2)), W(3));   % Compute inclination
OMEGA = atan2(W(1), -W(2));                         % Right ascension of ascending node                     % Compute RAAN
p     = norm(h)*norm(h)/mu;                         % Semi-latus rectum
a     = 1.0/(2.0/norm(r) - norm(v)*norm(v)/mu);     % Semi-major axis
n     = sqrt(mu/(a^3));                             % Mean motion

% Numerical stability hack for circular and near-circular orbits
% Ensures that (1-p/a) is always positive
if abs(a - p) < 1e-12 || abs((a - p)/a) < 1e-12
    p = a;
end

e     = sqrt(1 - p/a);                              % Eccentricity
E     = atan2(dot(r, v)/(n*a^2), (1-norm(r)/a));    % Eccentric Anomaly
M     = ecc2mean(E, e);                             % Mean Anomaly
u     = atan2(r(3), -r(1)*W(2) + r(2)*W(1));        % Mean longiude
nu    = atan2(sqrt(1-e*e)*sin(E), cos(E)-e);        % True Anomaly
omega = u - nu;                                     % Argument of perigee

% Correct angles to run from -PI to PI
i     = wrapToPi(i);
OMEGA = wrapToPi(OMEGA);
omega = wrapToPi(omega);
M     = wrapToPi(M);

% Create Orbital Element Vector
coe = [a, e, i, OMEGA, omega, M]';
end
