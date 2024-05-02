function shape = create_shape_cylinder(R, H, pos, rot, R1)
% Create a 3D cylinder shape. Dimensions in meters [m].
% - R: outter radius
% - H: height
% - pos: position (3x1)
% - rot: rotation matrix (3x3)
% - R1: inner radius (optional)

if nargin < 5
    R1 = 0;
end
R2 = R;

dtheta = 2*pi/20;
theta = [0:dtheta:2*pi-dtheta];
N = length(theta);


shape.Vertices(1,:) = [0 0 -H/2];
shape.Vertices(2,:) = [0 0 +H/2];
i_v = 3;
i_f = 1;
for i=1:N
    shape.Vertices(i_v,:) = [R*cos(theta(i)) R*sin(theta(i)) -H/2];
    i_v = i_v + 1;
    shape.Vertices(i_v,:) = [R*cos(theta(i)) R*sin(theta(i)) +H/2];
    
    if i > 1
        shape.Faces(i_f,:) = [i_v-1; i_v; i_v+1]-2;
        i_f = i_f + 1;
        shape.Faces(i_f,:) = [i_v; i_v+1; i_v+2]-2;
        i_f = i_f + 1;
        shape.Faces(i_f,:) = [3; i_v-1; i_v+1]-2;
        i_f = i_f + 1;
    end
    i_v = i_v + 1;
end
% Last faces
shape.Faces(i_f,:)   = [1; 3; i_v-2];
shape.Faces(i_f+1,:) = [3; 4; i_v-1];
shape.Faces(i_f+2,:) = [3; i_v-1; i_v-2];

% Moment of inertia (https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors)
% Solid cylinder of radius r, height h and mass m
I_over_m = diag([
    1/12 * (3 * (R2^2 + R1^2) + H^2);
    1/12 * (3 * (R2^2 + R1^2) + H^2);
    1/2 * (R2^2 + R1^2)]);

shape.Vertices = shape.Vertices * rot' + pos';
shape.r_CM = pos;
shape.rot = rot;
shape.I_over_m = rot * I_over_m * rot';
shape.volume = pi * (R2^2 - R1^2) * H;

end