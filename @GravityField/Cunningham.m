function [a, V, W] = Cunningham (obj, Position, nstart, varargin)
% varargin{1} is the user-provided degree of the subset size to be used.

% Position vector (Rotating Frame) [km]
x = Position(1);
y = Position(2);
z = Position(3);
r = norm(Position);

% Non-dimensional position vector
R = obj.R / r;
x = x/r;
y = y/r;
z = z/r;

% Figure out degree of the subset to be used in calculations
if length(obj.Subset) == 1
    N = obj.Subset;
    M = obj.Subset;
else
    N = obj.Subset(1);
    M = obj.Subset(2);
end
if nargin > 3  &&  all(varargin{1} >= 0)  &&  all(varargin{1} <= obj.Size)
    if length(varargin{1}) == 1
        N = varargin{1};
        M = varargin{1};
    else
        N = varargin{1}(1);
        M = varargin{1}(2);
    end
end

% If n is negative, return null acceleration
if N < 0
    a = zeros(3,1);
    return
end


% Allocate V and W arrays
V = zeros (N+2+1, N+2+1);
W = zeros (N+2+1, N+2+1);


% Initialize V00 and W00
V(0+2, 0+2) = R;
W(0+2, 0+2) = 0;


% Diagonal Recurrence
for i = 1:N+1
    V(i+2, i+2) = (2*i-1) * R * ( x*V(i-1+2, i-1+2) - y*W(i-1+2, i-1+2) );
    W(i+2, i+2) = (2*i-1) * R * ( x*W(i-1+2, i-1+2) + y*V(i-1+2, i-1+2) );
end


% Vertical Recurrence
for j = 0:M+1
    for i = j+1:N+1
        V(i+2, j+2) = R/(i-j) * ( (2*i-1) * z * V(i-1+2, j+2) - (i+j-1) * R * V(i-2+2, j+2) );
        W(i+2, j+2) = R/(i-j) * ( (2*i-1) * z * W(i-1+2, j+2) - (i+j-1) * R * W(i-2+2, j+2) );
    end
end


% Gravitational Acceleration (in Body Frame coords.) [km/s^2]
a = zeros(3,1);

if nstart == 0
    a(1) = a(1) - 2 * V(1+2, 1+2);
    a(2) = a(2) - 2 * W(1+2, 1+2);
    a(3) = a(3) - 2 * V(1+2, 0+2);
end

for i = 1:N
    a(1) = a(1) + 2 * obj.J(i) * V(i+1+2, 1+2);
    a(2) = a(2) + 2 * obj.J(i) * W(i+1+2, 1+2);
    a(3) = a(3) + 2 * obj.J(i) * V(i+1+2, 0+2) * (i+1);
end

for j = 1:M
    for i = j:N
        a(1) = a(1)         + ( -obj.C(i,j) * V(i+1+2, j+1+2) - obj.S(i,j) * W(i+1+2, j+1+2) ) + ...
            (i-j+2)*(i-j+1) * ( +obj.C(i,j) * V(i+1+2, j-1+2) + obj.S(i,j) * W(i+1+2, j-1+2) );
        a(2) = a(2)         + ( -obj.C(i,j) * W(i+1+2, j+1+2) + obj.S(i,j) * V(i+1+2, j+1+2) ) + ...
            (i-j+2)*(i-j+1) * ( -obj.C(i,j) * W(i+1+2, j-1+2) + obj.S(i,j) * V(i+1+2, j-1+2) );
        a(3) = a(3)         + ...
                  2*(i-j+1) * ( -obj.C(i,j) * V(i+1+2, j+2)   - obj.S(i,j) * W(i+1+2, j+2)   );
    end
end

a = a * 0.5 * obj.GM / obj.R^2;

end
