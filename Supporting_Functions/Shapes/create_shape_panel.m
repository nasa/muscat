function shape = create_shape_panel(L, W, pos, rot)
% Create a 2D panel shape. Dimensions in meters [m].
% - L: length
% - W: width
% - H: height
% - pos: position vector (3x1) [m]
% - rot: rotation matrix (3x3)

if nargin < 4
    rot = eye(3);
end

if nargin < 3
    pos = zeros(3,1);
end

H = 0.05;

shape = [];
shape.Vertices = [
    -1 -1  0;
    -1 +1  0;
    +1 +1  0;
    +1 -1  0] .* [L/2 W/2 0];

shape.Faces = [1 2 3; 1 3 4];

% Moment of inertia (https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors)
% Solid cuboid of length L, width W, height H, and mass m
I_over_m = diag([
    1/12 * (W^2 + H^2);
    1/12 * (L^2 + H^2);
    1/12 * (L^2 + W^2)]);

shape.Vertices = shape.Vertices * rot' + pos';
shape.r_CM = pos;
shape.rot = rot;
shape.I_over_m = rot * I_over_m * rot';
shape.volume = L * W * H;

end