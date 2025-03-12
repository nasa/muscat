function equatorial = Ecliptic2Equatorial(ecliptic)
% Input:
%  ecliptic: [l, b, r] (Nx3) or (3x1)
% Output:
%  equatorial: [ra, dec, R] (Nx3) or (3x1)
equatorial = VecFcn(@Ecliptic2Equatorial_, ecliptic);
end

function [ra, dec, R] = Ecliptic2Equatorial_(l, b, r)
% Convert ecliptic coordinates to equatorial coordinates
% Input:
%  ecliptic: [l, b, r] (3x1)
% Output:
%  equatorial: [ra, dec, R] (3x1)
% References:
%  https://aas.aanda.org/articles/aas/full/1998/01/ds1449/node3.html
%  https://en.wikipedia.org/wiki/Ecliptic_coordinate_system
%  https://en.wikipedia.org/wiki/Equatorial_coordinate_system

epsilon = 23.43929111; % obliquity of the ecliptic [deg]
lambda = l;
beta = b;

sin_delta = sind(beta) * cosd(epsilon) + cosd(beta) * sind(epsilon) * sind(lambda);
delta = asind(sin_delta);

cos_alpha = cosd(lambda) * cosd(beta) / cosd(delta);
sin_alpha = (-sind(beta) * sind(epsilon) + cosd(beta) * cosd(epsilon) * sind(lambda)) / cosd(delta);
alpha = atan2d(sin_alpha, cos_alpha);

ra = alpha;
dec = delta;
if nargout > 2
    R = r;
end


end