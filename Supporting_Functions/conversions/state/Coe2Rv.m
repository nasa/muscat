function rv = Coe2Rv(oe, mu)
rv = VecFcn(@Coe2Rv_, oe, mu);
end

function rv = Coe2Rv_(oe, mu)

% coe  : Classical OE
%       (6 x 1)
%       [a,  e,   i,   O,   w,   M]'
%       [km, -, rad, rad, rad, rad]'

a = oe(1);
e = oe(2);
i = oe(3);
O = oe(4);
w = oe(5);
M = oe(6);

E = Mean2Eccentric(M, e);

% Create perifocal coordinate vectors
P    = zeros(3,1);
P(1) = cos(w)*cos(O) - sin(w)*cos(i)*sin(O);
P(2) = cos(w)*sin(O) + sin(w)*cos(i)*cos(O);
P(3) = sin(w)*sin(i);

Q    = zeros(3,1);
Q(1) = -sin(w)*cos(O) - cos(w)*cos(i)*sin(O);
Q(2) = -sin(w)*sin(O) + cos(w)*cos(i)*cos(O);
Q(3) =  cos(w)*sin(i);

% Find 3-Dimensional Position
rv = zeros(6,1);
rv(1:3) = a*(cos(E)-e)*P + a*sqrt(1-e*e)*sin(E)*Q;
rv(4:6) = sqrt(mu*a)/norm(rv(1:3))*(-sin(E)*P + sqrt(1-e*e)*cos(E)*Q);
end
