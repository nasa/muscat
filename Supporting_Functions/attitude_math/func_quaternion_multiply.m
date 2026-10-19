function qprod = func_quaternion_multiply(q1, q2)
% Computes the product of two quaternions in the Breckenridge (JPL) convention.
% q = [qx, qy, qz, qw]

v1 = q1(1:3);
w1 = q1(4);
v2 = q2(1:3);
w2 = q2(4);

% The JPL cross-product term is the negative of the Hamilton term
vector_part = -cross(v1, v2) + w1*v2 + w2*v1;
scalar_part = w1*w2 - dot(v1, v2);

qprod = func_quaternion_properize([vector_part(:).', scalar_part]);
end