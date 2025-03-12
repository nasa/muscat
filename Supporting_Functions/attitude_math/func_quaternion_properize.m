function q_proper = func_quaternion_properize(q)
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

q_proper = q_proper / norm(q_proper);
end
