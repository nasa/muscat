function [a, V, W, dadr] = Cunningham_Derivatives (obj, Position, nstart, varargin)
    % varargin{1} is the user-provided degree of the subset size to be used.
    
    
    % Position vector (Rotating Frame) [km]
    x = Position(1);
    y = Position(2);
    z = Position(3);
    r = norm(Position);
    
    
    % Non-dimensional position vector
    R = obj.R / r;
    x_norm = x/r;
    y_norm = y/r;
    z_norm = z/r;
    
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
    for i = 1:N+1+1 % additional +1 for the derivative
        V(i+2, i+2) = (2*i-1) * R * ( x_norm*V(i-1+2, i-1+2) - y_norm*W(i-1+2, i-1+2) );
        W(i+2, i+2) = (2*i-1) * R * ( x_norm*W(i-1+2, i-1+2) + y_norm*V(i-1+2, i-1+2) );
    end
    
    
    % Vertical Recurrence
    for j = 0:N+1+1 % additional +1 for the derivative
        for i = j+1:N+1+1
            V(i+2, j+2) = R/(i-j) * ( (2*i-1) * z_norm * V(i-1+2, j+2) - (i+j-1) * R * V(i-2+2, j+2) );
            W(i+2, j+2) = R/(i-j) * ( (2*i-1) * z_norm * W(i-1+2, j+2) - (i+j-1) * R * W(i-2+2, j+2) );
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
    
    for i = 1:N
        for j = 1:M
            a(1) = a(1)         + ( -obj.C(i,j) * V(i+1+2, j+1+2) - obj.S(i,j) * W(i+1+2, j+1+2) ) + ...
                (i-j+2)*(i-j+1) * ( +obj.C(i,j) * V(i+1+2, j-1+2) + obj.S(i,j) * W(i+1+2, j-1+2) );
            a(2) = a(2)         + ( -obj.C(i,j) * W(i+1+2, j+1+2) + obj.S(i,j) * V(i+1+2, j+1+2) ) + ...
                (i-j+2)*(i-j+1) * ( -obj.C(i,j) * W(i+1+2, j-1+2) + obj.S(i,j) * V(i+1+2, j-1+2) );
            a(3) = a(3)         + ...
                      2*(i-j+1) * ( -obj.C(i,j) * V(i+1+2, j+2)   - obj.S(i,j) * W(i+1+2, j+2)   );
        end
    end
    
    a = a * 0.5 * obj.GM / obj.R^2;
    

    if nargout > 3

        daxdx = 0;
        daxdy = 0;
        daxdz = 0;
        daydz = 0;
        dazdz = 0;

        if nstart == 0
            daxdx = obj.GM / r^5 * (3*x^2 - r^2);
            daxdy = obj.GM / r^5 * (3*x*y);
            daxdz = obj.GM / r^5 * (3*x*z);
            daydz = obj.GM / r^5 * (3*y*z);
            dazdz = obj.GM / r^5 * (3*z^2 - r^2);
        end
        
        % daxdx
        for i = 1:N
            % m = 0
            daxdx = daxdx + obj.GM / obj.R^3 * (1/2) * ...
                        ((-obj.J(i) * V(i+2+2, 2+2)) - (i+2) * (i+1) * (-obj.J(i) * V(i+2+2, 0+2)));
            % m = 1
            if M > 0
                daxdx = daxdx + obj.GM / obj.R^3 * (1/4) * ( ...
                    (  +obj.C(i,1) * V(i+2+2, 3+2) + obj.S(i,1) * W(i+2+2, 3+2)) + ...
                    (i+1) * i * ...
                    (-3*obj.C(i,1) * V(i+2+2, 1+2) - obj.S(i,1) * W(i+2+2, 1+2)));
            end
            % m > 1
            for j = 2:M
                daxdx = daxdx + obj.GM / obj.R^3 * (1/4) * ( ...
                    (+obj.C(i,j) * V(i+2+2, j+2+2) + obj.S(i,j) * W(i+2+2, j+2+2)) + ...
                    2 * (i-j+2) * (i-j+1) * ...
                    (-obj.C(i,j) * V(i+2+2, j+0+2) - obj.S(i,j) * W(i+2+2, j+0+2)) + ...
                    (i-j+4) * (i-j+3) * (i-j+2) * (i-j+1) * ...
                    (+obj.C(i,j) * V(i+2+2, j-2+2) + obj.S(i,j) * W(i+2+2, j-2+2)));
            end
        end

        % daxdy
        for i = 1:N 
            % m = 0
            daxdy = daxdy + obj.GM / obj.R^3 * (1/2) * ...
                ((-obj.J(i) * W(i+2+2, 2+2)));
            % m = 1
            if M > 0
                daxdy = daxdy + obj.GM / obj.R^3 * (1/4) * ...
                    ((+obj.C(i,1) * W(i+2+2, 3+2) - obj.S(i,1) * V(i+2+2, 3+2)) + ...
                     (i+1) * i * ...
                     (-obj.C(i,1) * W(i+2+2, 1+2) - obj.S(i,1) * V(i+2+2, 1+2)));
            end
            % m > 1
            for j = 2:M
                daxdy = daxdy + obj.GM / obj.R^3 * (1/4) * ( ...
                    ((+obj.C(i,j) * W(i+2+2, j+2+2) - obj.S(i,j) * V(i+2+2, j+2+2)) + ...
                     (i-j+4) * (i-j+3) * (i-j+2) * (i-j+1) * ...
                     (-obj.C(i,j) * W(i+2+2, j-2+2) + obj.S(i,j) * V(i+2+2, j-2+2))));
            end
        end

        % daxdz
        for i = 1:N
            % m = 0
            daxdz = daxdz + obj.GM / obj.R^3 * ...
                ((i+1) * (-obj.J(i) * V(i+2+2, 1+2)));
            % m > 0
            for j = 1:M
                daxdz = daxdz + obj.GM / obj.R^3 * ( ...
                    (i-j+1)/2 * ...
                    (+obj.C(i,j) * V(i+2+2, j+1+2) + obj.S(i,j) * W(i+2+2, j+1+2)) + ...
                    (i-j+3) * (i-j+2) * (i-j+1) / 2 * ...
                    (-obj.C(i,j) * V(i+2+2, j-1+2) - obj.S(i,j) * W(i+2+2, j-1+2)));
            end
        end

        % daydz
        for i = 1:N
            % m = 0
            daydz = daydz + obj.GM / obj.R^3 * ...
                ((i+1) * (-obj.J(i) * W(i+2+2, 1+2)));
            % m > 0
            for j = 1:M
                daydz = daydz + obj.GM / obj.R^3 * ( ...
                    (i-j+1)/2 * ...
                    (+obj.C(i,j) * W(i+2+2, j+1+2) - obj.S(i,j) * V(i+2+2, j+1+2)) + ...
                    (i-j+3) * (i-j+2) * (i-j+1) / 2 * ...
                    (+obj.C(i,j) * W(i+2+2, j-1+2) - obj.S(i,j) * V(i+2+2, j-1+2)));
            end
        end

        % dazdz
        for i = 1:N
            % m = 0
            j = 0;
            dazdz = dazdz + obj.GM / obj.R^3 * ( ...
                    (i-j+2) * (i-j+1) * ...
                    (-obj.J(i) * V(i+2+2, j+2)));
            for j = 1:M 
                dazdz = dazdz + obj.GM / obj.R^3 * ( ...
                    (i-j+2) * (i-j+1) * ...
                    (+obj.C(i,j) * V(i+2+2, j+2) + obj.S(i,j) * W(i+2+2, j+2)));
            end
        end

        daydy = -daxdx - dazdz;

        dadr = [
            daxdx, daxdy, daxdz;
            daxdy, daydy, daydz;
            daxdz, daydz, dazdz;
        ];
    end
    end
    