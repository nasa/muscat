function sph = Cartesian2Spherical(cart)
% Input:
%   cart: Cartesian coordinates (Nx3) or (3x1)
% Output:
%   sph: Spherical coordinates [r, theta, phi] (Nx3) or (3x1)
% Note:
%   theta: polar angle, 0 <= theta <= pi
%   phi: azimuthal angle, 0 <= phi <= 2*pi

if size(cart, 1) == 3 && size(cart, 2) == 1
    r = norm(cart);
    theta = acos(cart(3)/r);
    phi = atan2(cart(2), cart(1));
    sph = [r; theta; phi];
else
    r = sqrt(sum(cart.^2, 2));
    theta = acos(cart(:, 3)./r);
    phi = wrapTo2Pi(atan2(cart(:, 2), cart(:, 1)));
    sph = [r, theta, phi];
end
end