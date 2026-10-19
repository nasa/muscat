function C = simpleDcm(axis,angleRad)
% SIMPLEDCM creates a single-axis direction cosine matrix for a given
% axis/angle
%
% Syntax:
%   C = simpleDcm(axis,angleRad)
%
% Inputs:
%   axis - Rotation axis number (1, 2, or 3)
%   angleRad - [rad] Rotation angle
%
% Outputs:
%   C - Direction cosine matrix

if ~ismember(axis,1:3)
    error('Invalid input axis %d. Must be 1, 2, or 3.',axis);
end

switch axis
    case 1
        C = [1 0 0; 0 cos(angleRad) sin(angleRad); 0 -sin(angleRad) cos(angleRad)];
    case 2
        C = [cos(angleRad) 0 -sin(angleRad); 0 1 0; sin(angleRad) 0 cos(angleRad)];
    case 3
        C = [cos(angleRad) sin(angleRad) 0; -sin(angleRad) cos(angleRad) 0; 0 0 1];
end