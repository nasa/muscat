function [a, V, W, dadr] = Acceleration (obj, Position, varargin)
% varargin{1} is the user-provided degree of the subset size to be used.
    
    % Perturbing Gravitational Acceleration in Body-Centric, Body-Fixed Frame coords. [km/s^2]
    if nargout > 3
        [a, V, W, dadr] = obj.Cunningham_Derivatives (Position, 0, varargin{:});
    else
        [a, V, W] = obj.Cunningham (Position, 0, varargin{:});
    end
end
