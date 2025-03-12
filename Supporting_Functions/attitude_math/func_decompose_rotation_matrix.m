function [a, phi] = func_decompose_rotation_matrix(C)
% phi is in radians

if abs(det(C) - 1) > 1e-10
    warning('det(C) not equal to 1!')
end

if norm(eye(3) - C*C') > 1e-10
    warning('inv(C) not equal to C^T !')
end

[V,D] = eig(C);

if norm(D(1,1) - 1) < 1e-10
    a = V(:,1);
elseif norm(D(2,2) - 1) < 1e-10
    a = V(:,2);
elseif norm(D(3,3) - 1) < 1e-10
    a = V(:,3);
else
    warning('eig(C) is not working!')
end

phi = acos( (trace(C)-1)/2 );

C_new1 = func_create_rotation_matrix(a, phi);
C_new2 = func_create_rotation_matrix(a, -phi);
C_new3 = func_create_rotation_matrix(-a, phi);
C_new4 = func_create_rotation_matrix(-a, -phi);

% if (norm(C_new1 - C) < 1e-8) && (phi > 0) 
%     % Do nothing
% elseif (norm(C_new2 - C) < 1e-8) && (phi < 0)
%     phi = -phi;
% elseif (norm(C_new3 - C) < 1e-8) && (phi > 0)
%     a = -a;
% elseif (norm(C_new4 - C) < 1e-8) && (phi < 0)
%     phi = -phi;
%     a = -a;
% else
%     error('C is not decomposed correctly!')
% end


if (norm(C_new1 - C) < 1e-5) && (a(1) >= 0)
    % Do nothing
elseif (norm(C_new2 - C) < 1e-5) && (a(1) >= 0)
    phi = -phi;
elseif (norm(C_new3 - C) < 1e-5) && (a(1) <= 0)
    a = -a;
elseif (norm(C_new4 - C) < 1e-5) && (a(1) <= 0)
    phi = -phi;
    a = -a;
else
    error('C is not decomposed correctly!')
end

phi = real(phi);