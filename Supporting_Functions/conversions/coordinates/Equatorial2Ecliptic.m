function [l,b,r] = Equatorial2Ecliptic(ra, dec, R)
% Convert equatorial coordinates to ecliptic coordinates
% Input:
%  ra = right ascension [deg]
%  dec = declination [deg]
%  R = distance [any]
% Output:
%  l = ecliptic longitude [deg]
%  b = ecliptic latitude [deg]
%  r = distance [same R]
% References:
%  https://aas.aanda.org/articles/aas/full/1998/01/ds1449/node3.html
%  https://en.wikipedia.org/wiki/Ecliptic_coordinate_system
%  https://en.wikipedia.org/wiki/Equatorial_coordinate_system

epsilon = 23.43929111; % obliquity of the ecliptic [deg]
alpha = ra;
delta = dec;

sin_beta = sind(delta) * cosd(epsilon) - cosd(delta) * sind(epsilon) * sind(alpha);
beta = asind(sin_beta);

cos_lambda = cosd(alpha) * cosd(delta) / cosd(beta);
sin_lambda = (sind(delta) * sind(epsilon) + cosd(delta) * cosd(epsilon) * sind(alpha)) / cosd(beta);
lambda = atan2d(sin_lambda, cos_lambda);

l = lambda;
b = beta;
if nargout > 2
    r = R;
end
end



