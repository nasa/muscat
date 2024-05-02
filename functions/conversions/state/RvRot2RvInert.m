function rv_inert = RvRot2RvInert(rv_rot, rv_origin)
% RvRot2RvInert - Transforms a relative position and velocity vector from
% the rotating reference system to the inertial reference system. Both
% referemce systems have the same frame (axes).
% rv_origin  6 x 1 [km, km/s]
% rv_rot     6 x 1 [km, km/s]
% rv_inert   6 x 1 [km, km/s]


% Rotation matrix from inertial to rotating frame
R_rot_inert = eye(3); % Same frame (e.g., J2000)

% Angular velocity of rotating frame
r_origin = rv_origin(1:3);
v_origin = rv_origin(4:6);
h = cross(r_origin, v_origin);
omega = h / norm(r_origin)^2;

% Relative position and velocity in rotating frame
r_rel_inert = rotation_matrix' * rv_rot(1:3);
v_rel_inert = rotation_matrix' * rv_rot(4:6) + cross(omega, r_rel_inert);

rv_inert = zeros(6,1);
rv_inert(1:3) = rv_origin(1:3) + R_rot_inert * rv_rot(1:3);
rv_inert(4:6) = v_inert;

end