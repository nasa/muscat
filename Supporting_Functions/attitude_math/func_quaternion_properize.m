function q_proper = func_quaternion_properize(q)

%% JPL Convention : Scalar part is positive, and it is the 4th term
if q(4) < 0
    q_proper = -q;
elseif q(4) == 0
    if q(1) < 0
        q_proper = -q;
    else
        q_proper = q;
    end
else
    q_proper = q;
end

%% Normalize the quaternion
q_proper = q_proper / norm(q_proper);

end
