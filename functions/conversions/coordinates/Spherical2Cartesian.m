function cart = Spherical2Cartesian(sph)
% Input:
%   sph: Spherical coordinates [r, theta, phi] (Nx3) or (3x1)
% Output:
%   cart: Cartesian coordinates (Nx3) or (3x1)
% Note:
%   theta: polar angle, 0 <= theta <= pi
%   phi: azimuthal angle, 0 <= phi <= 2*pi

if size(sph, 1) == 3 && size(sph, 2) == 1
    r = sph(1);
    theta = sph(2);
    phi = sph(3);
    cart = [
        r*sin(theta)*cos(phi);
        r*sin(theta)*sin(phi);
        r*cos(theta)];
else
    r = sph(:, 1);
    theta = sph(:, 2);
    phi = sph(:, 3);
    cart = [
        r.*sin(theta).*cos(phi);
        r.*sin(theta).*sin(phi);
        r.*cos(theta)];
end
end