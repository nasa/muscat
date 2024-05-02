function cart = Planetocentric2Cartesian(rlatlon)
% Input:
%   rlatlon: Planetocentric coordinates [r, lat, lon], [rad] (Nx3) or (3x1)
% Output:
%   cart: Cartesian coordinates (Nx3) or (3x1)
% Note:
%   lat: latitude, -pi/2 <= lat <= pi/2
%   lon: longitude, -pi <= lon <= pi

if size(rlatlon, 1) == 3 && size(rlatlon, 2) == 1
    r = rlatlon(1);
    lat = rlatlon(2);
    lon = rlatlon(3);
    cart = [r*cos(lat)*cos(lon); r*cos(lat)*sin(lon); r*sin(lat)];
else
    cart = zeros(size(rlatlon));
    r = rlatlon(:, 1);
    lat = rlatlon(:, 2);
    lon = rlatlon(:, 3);
    cart(:, 1) = r.*cos(lat).*cos(lon);
    cart(:, 2) = r.*cos(lat).*sin(lon);
    cart(:, 3) = r.*sin(lat);
end
end