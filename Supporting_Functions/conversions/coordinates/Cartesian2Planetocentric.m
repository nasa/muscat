function rlatlon = Cartesian2Planetocentric(cart)
% Input:
%   cart: Cartesian coordinates (Nx3) or (3x1)
% Output:
%   sph: Planetocentric coordinates [r, lat, lon] [rad] (Nx3) or (3x1)
% Note:
%   lat: latitude, -pi/2 <= lat <= pi/2
%   lon: longitude, -pi <= lon <= pi

if size(cart,1) == 3 && size(cart,2) == 1
    r = norm(cart);
    lat = asin(cart(3)/r);
    lon = atan2(cart(2), cart(1));
    rlatlon = [r; lat; lon];
else
    r = sqrt(sum(cart.^2,2));
    lat = asin(cart(:,3)./r);
    lon = atan2(cart(:,2), cart(:,1));
    rlatlon = [r, lat, lon];
end
end