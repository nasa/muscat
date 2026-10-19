function x = crossv(y,z)
% function x = crossv(y,z)
% 
% cross product for 3-vectors
% works with:
%   column or row vectors, shape is shape of y
%   y vector, z matrix of (row) vectors
%   y and z matching set of row vectors, result is row-by-row product

% 8/95 sws initial
% 2/17/2013 enforce sizes

[o,p] = size(y);
[m,n] = size(z);

if ( (m==1) | (n==1) )
  % z a single vector
  if length(z) ~= 3
    error('crossv: need z(3)')
  end
  if ( (o==1) | (p==1) )
    % both are just vectors
    % require both are length 3
    if length(y) ~= 3
      error('crossv: need y(3)')
    end
    % take the shape of y
    x = zeros(size(y));
    x(1) = y(2)*z(3)-y(3)*z(2);
    x(2) = y(3)*z(1)-y(1)*z(3);
    x(3) = y(1)*z(2)-y(2)*z(1);
  else
    % more than one vector in y
    % assume p == 3
    if p ~= 3
      error('crossv: need y(m,3)')
    end
    if (n == 1)
      % transpose to get row vector
      z = z';
    end
    % use size of y
    x = y;
    for i=1:o
      x(i,:) = crossv(y(i,:),z) ;
    end
  end
else
  % more than 1 vector in z
  % require n = 3
  if n ~= 3
    error('crossv: need z(m,3)')
  end
  % watch y shape
  if ( (o==1) | (p==1) )
    % y is a vector
    if (p == 1)
      % transpose to get row vector
      y = y';
    end
    % use size of z
    x=z;
    for i=1:m
      x(i,:) = crossv(y,z(i,:)) ;
    end
  else
    % more than 1 vector in y
    % assume matches shape of z
    if o ~= m
      error('crossv: dimension missmatch')
    end
    x = z;
    for i=1:m
      x(i,:) = crossv(y(i,:), z(i,:));
    end
  end
end

% end
