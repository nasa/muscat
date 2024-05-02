function C = func_create_Rot_matrix(a, phi)

if abs(norm(a) - 1) > 1e-10
    warning('Norm a is not equal to 1!')
end

if (size(a,1) == 3) && (size(a,2) == 1)
    % Do nothing
else
    error('Size of a is incorrect!')
end

skew_a = skew(a);

C = cos(phi)*eye(3) + (1-cos(phi))*a*a' + sin(phi)*skew_a;