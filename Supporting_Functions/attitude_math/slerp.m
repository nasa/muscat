function q = slerp(q1, q2, t)
% Ensure q1 and q2 are normalized
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Compute the cosine of the angle between the two vectors.
cosTheta = dot(q1, q2);

% If cosTheta < 0, the interpolation will be done using the shorter path.
if cosTheta < 0
    q1 = -q1;
    cosTheta = -cosTheta;
end

% If the inputs are too close, linearly interpolate to avoid numerical issues.
if cosTheta > 0.95
    q = (1-t)*q1 + t*q2;
    return;
end

% Compute the angle (sinTheta is always positive due to the normalization)
theta = acos(cosTheta);
sinTheta = sqrt(1 - cosTheta*cosTheta);

scale1 = sin((1 - t) * theta) / sinTheta;
scale2 = sin(t * theta) / sinTheta;

q = scale1 * q1 + scale2 * q2;
end