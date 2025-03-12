function eclipse = calculate_eclipse(pos_body, pos_target, pos_sc, body_radius)
if all(size(pos_target) == [3 1]) && all(size(pos_sc) == [3 1])
    % vector-vector
    eclipse = calculate_eclipse_(pos_body, pos_target, pos_sc, body_radius);
elseif size(pos_target, 2) == 3 && all(size(pos_sc) == [3,1])
    % matrix-vector
    eclipse = zeros(size(pos_target, 1), 1);
    for i = 1:size(pos_target, 1)
        eclipse(i) = calculate_eclipse_(pos_body, pos_target(i,:)', pos_sc, body_radius);
    end
elseif all(size(pos_target) == [3 1]) && size(pos_sc, 2) == 3
    % vector-matrix
    eclipse = zeros(size(pos_sc, 1), 1);
    for i = 1:size(pos_sc, 1)
        eclipse(i) = calculate_eclipse_(pos_body, pos_target, pos_sc(i,:)', body_radius);
    end
elseif size(pos_target, 2) == 3 && size(pos_sc, 2) == 3
    % matrix-matrix
    eclipse = zeros(size(pos_target, 1), 1);
    for i = 1:size(pos_target, 1)
        eclipse(i) = calculate_eclipse_(pos_body, pos_target(i,:)', pos_sc(i,:)', body_radius);
    end
else
    error('Input size is not correct')
end
end

function eclipse = calculate_eclipse_(pos_body, pos_target, pos_sc, body_radius)
% Eclipsed by Body
% Use https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
x0 = pos_body(:);
x1 = pos_target(:);
x2 = pos_sc(:);

distance = norm( cross(x0-x1, x0-x2) )/ norm(x2-x1);

eclipse = 0;
if distance > body_radius
    % No eclipse
else
    % Check t
    t = - dot( x1-x0, x2-x1 )/(norm(x2-x1))^2;
    if (t>=0) && (t<=1)
        % Eclipsed by Earth
        eclipse = 1;
    end
end
end