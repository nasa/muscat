% Calculate the normal vector of a triangle

function unit_normal = func_triangleNormal(P1, P2, P3)
    % Compute edge vectors
    v1 = P2 - P1;
    v2 = P3 - P1;

    % Calculate unnormalized normal vector via cross product
    raw_normal = cross(v1, v2);

    % Normalize to unit length
    unit_normal = raw_normal / norm(raw_normal);
end