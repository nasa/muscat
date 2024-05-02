function [newAttitude,DCM] = ConvertAttitude(attitude,old,new)
%ConvertAttitde will convert between common attitude parameter sets.
%   NEWATTITUDE = ConvertAttitude(ATTITUDE) will assume ATTITUDE is
%   a proper Direction Cosine Matrix (DCM) and convert it to 3-2-1 Euler
%   angles. ATTITUDE must be a 3-by-3-by-n matrix where n is the number of
%   DCMs in the series. NEWATTITUDE will be a 3-by-n matrix.
%
%   [NEWATTITUDE, DCM] = ConvertAttitude(ATTITUDE,OLD) will convert
%   ATTITUDE to its corresponding DCM. ATTITUDE must be one of the
%   following attitude parameter sets:
%       - Direction Cosine Matrix (DCM), 3-by-3-by-n
%       - Euler Angles (all 12 sets are supported), 3-by-n, radians
%       - Quaternions (Euler Parameters), 4-by-n, scalar component first
%       - Principal Rotation Vector (PRV), 3-by-n
%       - Classical Rodrigues Parameters (CRP, Gibbs Vector), 3-by-n
%       - Modified Rodrigues Parameters (MRP), 3-by-n
%   If ATTITUDE is a DCM, then the default output is 3-2-1 Euler Angles.
%   OLD is the name of the input parameter set and must be a string. Case
%   and spacing do not matter. For instance, 'Euler Angles' and
%   'eulerangles' are both acceptable. Acronyms are also acceptable: 'mrp' 
%   'prv' 'crp' etc. For Euler Angles, OLD is specified by the rotation
%   sequence: '123' '313' '213' etc. DCM is the corresponding series of
%   DCMs and will be a 3-by-3-by-n matrix.
%
%   [NEWATTITUDE, DCM] = ConvertAttitude(...,NEW) will convert ATTITUDE
%   from its original parameter set OLD to the specified NEW parameter set.
%   NEW must follow the same rules as OLD.
%
%   EXAMPLE:  newAttitude = ConvertAttitude([.1; .2; .3], 'mrp', '321')
%           
%             newAttitude =
%                           1.3564
%                           0.3519
%                           0.7416
%
%   EXAMPLE:  newAttitude = ConvertAttitude([[.1; .2; .3],[.3; .2; .1]], 'mrp', '321')
%
%             newAttitude =
%                           1.3564    0.7416
%                           0.3519    0.3519
%                           0.7416    1.3564
%
%   Attitude equations were taken from "Analyical Mechanics of Space
%   Systems" by Junkins and Schaub, an excellent resource for further
%   information, such as singularities and redundancies associated with 
%   these attitude parameter sets.
%
%   Joshua Chabot 2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DCM orthonormal warning tolerance
tol = 1e-14;
%% Input Verification
% Default for 1 input is conversion from DCM to 3-2-1 Euler angles
if (nargin == 1)
    old = 'DCM';
    new = '321';
end
    
% Default for 2 inputs is conversion to DCM, unless already a DCM
if (nargin == 2)
    if strcmp(old,'dcm')
        new = '321';
    else
        new = 'dcm';
    end
end
% Check inputs are real
if (~isreal(attitude))
    error('Attitude parameters must be real')
end
% Check that parameter set inputs are strings
if (~ischar(old))||(~ischar(new))
    error('Attitude parameter tags must strings')
end
% Convert Strings to Lowercase
old = lower(old);
new = lower(new);
% Remove Spaces in Strings
old = regexprep(old,' ','');
new = regexprep(new,' ','');
% Rotation Matrices
ROT1 = @(t) [1   0 0;
             0   cos(t) sin(t);
             0  -sin(t) cos(t)];
ROT2 = @(t) [cos(t) 0 -sin(t);
             0      1  0;
             sin(t) 0  cos(t)];
ROT3 = @(t) [cos(t) sin(t) 0;
            -sin(t) cos(t) 0;
             0         0   1];
         
% Skew-Symmetric Matrix Operator
tilde = @(v) [0    -v(3)  v(2);
              v(3)  0    -v(1);
             -v(2)  v(1)  0];
if strcmp(old,'dcm')
    dim = size(attitude,3);
    flag = 1;
else
    dim = size(attitude,2);
    flag = 0;
end
DCM = zeros(3,3,dim);
for n = 1:dim
    if flag
        x = attitude(:,:,n);
    else
        x = attitude(:,n);
    end
%% Conversion to DCM
switch old
    % DCM
    case {'dcm','directioncosinematrix'}
        if (size(x,1) ~= 3)||(size(x,2) ~= 3)
            error('Input is not a DCM. Must be proper 3-by-3 rotation matrix.');
        end
        if (abs(det(x) - 1) > tol)
            warning('Determinant of DCM deviates from unity by at least +/- 1e-12.')
        end
        C = x;
        
    % Euler Angles
    case '121'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT1(x(3))*ROT2(x(2))*ROT1(x(1));
    case '123'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT3(x(3))*ROT2(x(2))*ROT1(x(1));
    case '131'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT1(x(3))*ROT3(x(2))*ROT1(x(1));
    case '132'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT2(x(3))*ROT3(x(2))*ROT1(x(1));
    case '212'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT2(x(3))*ROT1(x(2))*ROT2(x(1));
    case '213'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT3(x(3))*ROT1(x(2))*ROT2(x(1));
    case '231'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT1(x(3))*ROT3(x(2))*ROT2(x(1));
    case '232'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT2(x(3))*ROT3(x(2))*ROT2(x(1));
    case '312'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT2(x(3))*ROT1(x(2))*ROT3(x(1));
    case '313'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT3(x(3))*ROT1(x(2))*ROT3(x(1));
    case '321'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT1(x(3))*ROT2(x(2))*ROT3(x(1));
    case '323'
        if (size(x,1) ~= 3)
            error('Euler angle parameter set must be 3-by-n matrix.')
        end
        C = ROT3(x(3))*ROT2(x(2))*ROT3(x(1));
        
    % Principal Rotation Vector
    case {'principalrotationvector','prv','principalrotationparameter','prp'}
        if (size(x,1) ~= 3)
            error('Principal rotation vector parameter set must be 3-by-n matrix with principal rotation angle as the first component of each column.')
        end
        phi = norm(x);
        x = x/phi;
        S = 1 - cos(phi);
        C = [x(1)^2*S+cos(phi) x(1)*x(2)*S+x(3)*sin(phi) x(1)*x(3)*S-x(2)*sin(phi);...
             x(2)*x(1)*S-x(3)*sin(phi) x(2)^2*S+cos(phi) x(2)*x(3)*S+x(1)*sin(phi);...
             x(1)*x(3)*S+x(2)*sin(phi) x(2)*x(3)*S-x(1)*sin(phi) x(3)^2*S+cos(phi)];
           
    % Quaternions
    case {'quaternions','eulerparameters','ep','quaternion','eulerparameter','eps'}
        if (size(x,1) ~= 4)
            error('Quaternion parameter set must be 4-by-n matrix.')
        end
%         if (norm(x) ~= 1)
%             error('Quaternions must have unit norm.')
%         end
        C = [x(1)^2+x(2)^2-x(3)^2-x(4)^2 2*(x(2)*x(3)+x(1)*x(4)) 2*(x(2)*x(4)-x(1)*x(3));...
             2*(x(2)*x(3)-x(1)*x(4)) x(1)^2-x(2)^2+x(3)^2-x(4)^2 2*(x(3)*x(4)+x(1)*x(2));...
             2*(x(2)*x(4)+x(1)*x(3)) 2*(x(3)*x(4)-x(1)*x(2)) x(1)^2-x(2)^2-x(3)^2+x(4)^2];
    
    % Classical Rodrigues Parameters
    case {'rodriguesparameters','classicalrodriguesparameters','crp','crps','gibbs','gibbsvector'}
        if (size(x,1) ~= 3)
            error('Classical Rodrigues Parameters must be 3-by-n matrix.')
        end
        C = (1/(1+x'*x))*((1-x'*x)*eye(3)+2*x*x'-2*tilde(x));
        
    % Modified Rodrigues Parameters, MRP is such that >= 1
    case {'modifiedrodriguesparameters','mrp','mrps'}
        if (size(x,1) ~= 3)
            error('Modified Rodrigues Parameters must be 3-by-n matrix.')
        end
        C = eye(3)+(8*tilde(x)^2-4*(1-x'*x)*tilde(x))/(1+x'*x)^2;
    otherwise
        error('Invalid input attitude parameter set tag.')
end
        
%% Conversion to New Parameter Set
switch new
    case {'dcm','directioncosinematrix'}
        newAttitude(:,:,n) = C;
    case '121'
        newAttitude(:,n) = [atan2(C(1,2),-C(1,3));...
                            acos(C(1,1));...
                            atan2(C(2,1),C(3,1))];
    case '123'
        newAttitude(:,n) = [atan2(-C(3,2),C(3,3));...
                            asin(C(3,1));...
                            atan2(-C(2,1),C(1,1))];
    case '131'
        newAttitude(:,n) = [atan2(C(1,3),C(1,2));...
                            acos(C(1,1));...
                            atan2(C(3,1),-C(2,1))];
    case '132'
        newAttitude(:,n) = [atan2(C(2,3),C(2,2));...
                            asin(-C(2,1));...
                            atan2(C(3,1),C(1,1))];
    case '212'
        newAttitude(:,n) = [atan2(C(2,1),C(2,3));...
                            acos(C(2,2));...
                            atan2(C(1,2),-C(3,2))];
    case '213'
        newAttitude(:,n) = [atan2(C(3,1),C(3,3));...
                            asin(-C(3,2));...
                            atan2(C(1,2),C(2,2))];
    case '231'
        newAttitude(:,n) = [atan2(-C(1,3),C(1,1));...
                            asin(C(1,2));...
                            atan2(-C(3,2),C(2,2))];
    case '232'
        newAttitude(:,n) = [atan2(C(2,3),-C(2,1));...
                            acos(C(2,2));...
                            atan2(C(3,2),C(1,2))];
    case '312'
        newAttitude(:,n) = [atan2(-C(2,1),C(2,2));...
                            asin(C(2,3));...
                            atan2(-C(1,3),C(3,3))];
    case '313'
        newAttitude(:,n) = [atan2(C(3,1),-C(3,2));...
                            acos(C(3,3));...
                            atan2(C(1,3),C(2,3))];
    case '321'
        newAttitude(:,n) = [atan2(C(1,2),C(1,1));...
                           -asin(C(1,3));...
                            atan2(C(2,3),C(3,3))];
    case '323'
        newAttitude(:,n) = [atan2(C(3,2),C(3,1));...
                            acos(C(3,3));...
                            atan2(C(2,3),-C(1,3))];
    case {'principalrotationvector','prv','principalrotationparameters','prp'}
        phi = acos((1/2)*(trace(C)-1));
        newAttitude(:,n) = (phi/(2*sin(phi)))*[C(2,3) - C(3,2);...
                                               C(3,1) - C(1,3);...
                                               C(1,2) - C(2,1)];
                       
    case {'quaternion','quaternions','eulerparameters','ep'}
        newAttitude(:,n) = DCM2Quaternion(C);
    case {'rodriguesparameters','classicalrodriguesparameters','crp'}
        b = DCM2Quaternion(C);
        newAttitude(:,n) = [b(2)/b(1);...
                            b(3)/b(1);...
                            b(4)/b(1)];
    case {'modifiedrodriguesparameters','mrp'}
        b = DCM2Quaternion(C);
        newAttitude(:,n) = [b(2)/(1+b(1));...
                            b(3)/(1+b(1));...
                            b(4)/(1+b(1))];
    otherwise
        error('Invalid output attitude parameter set tag.')
end
DCM(:,:,n) = C;
end
function b = DCM2Quaternion(C)
% Converts DCM to quaternions via the Stanley Method
B2(1) = (1+trace(C))/4;
B2(2) = (1+2*C(1,1)-trace(C))/4;
B2(3) = (1+2*C(2,2)-trace(C))/4;
B2(4) = (1+2*C(3,3)-trace(C))/4;
[~,i] = max(B2);
switch i
	case 1
		b(1) = sqrt(B2(1));
		b(2) = (C(2,3)-C(3,2))/4/b(1);
		b(3) = (C(3,1)-C(1,3))/4/b(1);
		b(4) = (C(1,2)-C(2,1))/4/b(1);
	case 2
		b(2) = sqrt(B2(2));
		b(1) = (C(2,3)-C(3,2))/4/b(2);
		if (b(1)<0)
			b(2) = -b(2);
			b(1) = -b(1);
		end
		b(3) = (C(1,2)+C(2,1))/4/b(2);
		b(4) = (C(3,1)+C(1,3))/4/b(2);
	case 3
		b(3) = sqrt(B2(3));
		b(1) = (C(3,1)-C(1,3))/4/b(3);
		if (b(1)<0)
			b(3) = -b(3);
			b(1) = -b(1);
		end
		b(2) = (C(1,2)+C(2,1))/4/b(3);
		b(4) = (C(2,3)+C(3,2))/4/b(3);
	case 4
		b(4) = sqrt(B2(4));
		b(1) = (C(1,2)-C(2,1))/4/b(4);
		if (b(1)<0)
			b(4) = -b(4);
			b(1) = -b(1);
		end
		b(2) = (C(3,1)+C(1,3))/4/b(4);
		b(3) = (C(2,3)+C(3,2))/4/b(4);
end
b = b';
end
end
