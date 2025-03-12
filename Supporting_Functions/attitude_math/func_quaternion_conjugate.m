function qc = func_quaternion_conjugate(q)
% Computes the conjugate of a quaternion
qc = func_quaternion_properize([-q(1:3), q(4)]);
end
