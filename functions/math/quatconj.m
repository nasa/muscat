function qc = quatconj(q)
% Computes the conjugate of a quaternion
qc = quatproperize([-q(1:3); q(4)]);
end
