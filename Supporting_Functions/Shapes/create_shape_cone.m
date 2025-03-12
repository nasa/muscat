function shape = create_shape_cone(R, H, pos, rot)
% Create a 3D paraboloid shape. Dimensions in meters [m].
% - R: radius
% - H: height
% - pos: position (3x1)
% - rot: rotation matrix (3x3)

dtheta = 2*pi/20;
theta = [0:dtheta:2*pi-dtheta];
N = length(theta);

shape.Vertices = zeros(N+1,3);
shape.Vertices(1,:) = [0 0 -H/2];
shape.Vertices(2:end,:) = [R*cos(theta') R*sin(theta') H/2*ones(N,1)];

shape.Faces = zeros(N,3);
shape.Faces(:,1) = 1;
shape.Faces(:,2) = [2:N+1];
shape.Faces(:,3) = [3:N+1 2];

% Moment of inertia (https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors)
% Solid cylinder of radius r, height h and mass m
I_over_m = diag([
    3/5*H^2+3/20*R^2;
    3/5*H^2+3/20*R^2;
    3/10*R^2
    ]);

% Thickness of the walls
r = 0.98 * R;
h = 0.98 * H;

shape.Vertices = shape.Vertices * rot' + pos';
shape.r_CM = pos;
shape.rot = rot;
shape.I_over_m = rot * I_over_m * rot';
shape.volume = pi/3 * (R^2 * H - r^2 * h);

end