function theta_radians = func_angle_between_vectors(u,v)
theta_radians = VecFcn(@func_angle_between_vectors_, u, v);
end

function theta_radians = func_angle_between_vectors_(u,v)
% See this: https://www.mathworks.com/matlabcentral/answers/101590-how-can-i-determine-the-angle-between-two-vectors-in-matlab
if size(u) ~= size(v)
    error('Size error between u and v!')
end
theta_radians = atan2(norm(cross(u,v)),dot(u,v));
end