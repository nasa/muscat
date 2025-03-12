function c = VecFcn(fcn, a, b)
% Vectorize functions

if nargin == 2
    if size(a,1) > 1 && size(a,2) == 1
        % vector
        c = fcn(a);
    else
        % matrix
        c = zeros(size(a));
        for i = 1:size(a,1)
            c(i,:) = fcn(a(i,:)');
        end
    end
elseif nargin == 3
    if isequal(size(b), [1, 1])
        % b is a scalar
        if size(a,2) == 1
            % vector
            c = fcn(a, b);
        elseif size(a,2) == 6
            % matrix
            c = zeros(size(a));
            for i = 1:size(a,1)
                c(i,:) = fcn(a(i,:)', b)';
            end
        else
            error('Wrong inputs')
        end
    else
        if size(a,2) == 1 && size(b,2) == 1
            % vector - vector
            c = fcn(a, b);
        elseif size(a,2) ~= 1 && size(b,2) ~= 1
            % matrix - matrix
            c = zeros(size(a));
            for i = 1:size(a,1)
                c(i,:) = fcn(a(i,:)', b(i,:)')';
            end
        elseif size(a,2) == 1 && size(b,2) ~= 1
            % vector - matrix
            c = cell(size(b,1), 1);
            for i = 1:size(b,1)
                c{i} = fcn(a, b(i,:)')';
            end
            c = cell2mat(c);
        elseif size(a,2) ~= 1 && size(b,2) == 1
            % matrix - vector
            c_ = fcn(a(1,:)', b)';
            c = zeros(size(a,1), size(c_,2));
            for i = 1:size(a,1)
                c(i,:) = fcn(a(i,:)', b)';
            end
        else
            error('Wrong inputs')
        end
    end
end
end