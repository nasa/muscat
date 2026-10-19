function vAngle = v1v2angle(v1,v2)
%% v1v2angle computes the precise angle between two sets of vectors
% This function is a more precise way of computing angles between two
% vectors than using acos(dot(v1,v2)), which suffers precision loss for
% small angles. Instead, this function computes the angle as the arctangent
% of the norm of the cross product divided by the dot product of the two
% sets of vectors, i.e. vAngle = atan2(norm(cross(v1,v2)),dot(v1,v2)). This
% form has the added benefit that the two input vectors do not need to be
% normalized in order to get the correct answer.
%
% Syntax:
%   vAngle = v1v2angle(v1,v2)
%
% Inputs:
%   v1 - [nx3 or 3xn] Vector set 1
%   v2 - [nx3 or 3xn] Vector set 2
%   Note: If inputs are 3x3, this function assumes column vectors
%
% Outputs:
%   vAngle - [nx1 or 1xn] Angles between v1 and v2 (rad). The output
%       dimension will be consistent with the inputs

% Handle input dimensions and make sure they are consistent
v1Size = size(v1);
v2Size = size(v2);
if v1Size(2) == 3 && v1Size(1) ~= 3
    % v1 is nx3
    opDim = 2;
    if v2Size(2) ~= 3 && v2Size(1) == 3 && v2Size(2) == v1Size(1)
        % Check if v2 needs to be made the same size as v1
        v2 = v2.'; 
    elseif (v2Size(2) ~= 3 && v2Size(1) == 3 && v1Size(1) ~= v2Size(2)) || v1Size(1) ~= v2Size(1)
        error('Input vectors must be the same length.')
    end
else
    % v1 is 3xn
    opDim = 1;
    if v2Size(1) ~= 3 && v2Size(2) == 3 && v2Size(1) == v1Size(2)
        % Check if v2 needs to be made the same size as v1
        v2 = v2.';
    elseif (v2Size(1) ~= 3 && v2Size(2) == 3 && v1Size(2) ~= v2Size(1)) || v1Size(2) ~= v2Size(2)
        error('Input vectors must be the same length.')
    end
end

v1v2Cross = cross(v1,v2,opDim);
v1v2Dot = dot(v1,v2,opDim);
vAngle = atan2(vecnorm(v1v2Cross,2,opDim),v1v2Dot);