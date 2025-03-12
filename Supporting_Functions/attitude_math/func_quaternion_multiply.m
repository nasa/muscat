function qprod = func_quaternion_multiply(q1, q2)
% Computes the product of two quaternions
vector_part = cross(q1(1:3), q2(1:3)) + q1(4)*q2(1:3) + q2(4)*q1(1:3);
scalar_part = q1(4)*q2(4) - dot(q1(1:3), q2(1:3));
qprod = func_quaternion_properize([vector_part, scalar_part]);
end