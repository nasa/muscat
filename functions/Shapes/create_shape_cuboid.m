function shape = create_shape_cuboid(L, W, H, pos, rot)
    % Create a 3D cuboid shape with triangular faces. Dimensions in meters [m].
    % - L: length
    % - W: width
    % - H: height
    % - pos: position (3x1)
    % - rot: rotation matrix (3x3)

    % Define cube vertices
    shape.Vertices = [
        0   0   0;  % Vertex 1
        L   0   0;  % Vertex 2
        L   W   0;  % Vertex 3
        0   W   0;  % Vertex 4
        0   0   H;  % Vertex 5
        L   0   H;  % Vertex 6
        L   W   H;  % Vertex 7
        0   W   H   % Vertex 8
    ];
    shape.Vertices = shape.Vertices - [L/2 W/2 H/2];

    % Define cube faces as triangles
    shape.Faces = [
        1 2 3;  % Bottom face triangle 1
        1 3 4;  % Bottom face triangle 2
        5 6 7;  % Top face triangle 1
        5 7 8;  % Top face triangle 2
        1 2 6;  % Front face triangle 1
        1 6 5;  % Front face triangle 2
        2 3 7;  % Right face triangle 1
        2 7 6;  % Right face triangle 2
        3 4 8;  % Back face triangle 1
        3 8 7;  % Back face triangle 2
        4 1 5;  % Left face triangle 1
        4 5 8   % Left face triangle 2
    ];

    % Moment of inertia
    % Solid cube of side s and mass m
    m = H * W * L;  % Here m is essentially the volume, not the actual mass
    I_over_m = diag([
        1/12 * (H^2 + L^2);
        1/12 * (H^2 + W^2);
        1/12 * (L^2 + W^2)
    ]);

    % Apply rotation and translation to the cube vertices
    shape.Vertices = shape.Vertices * rot' + pos';

    shape.r_CM = pos;
    shape.rot = rot;
    shape.I_over_m = rot * I_over_m * rot';
    shape.volume = H * W * L;

end
