function qnsoe = Coe2QnsOe(coe)
    qnsoe = VecFcn(@Coe2QnsOe_, coe);
    end
    
function qnsoe = Coe2QnsOe_(coe)
    a = coe(1);
    e = coe(2);
    i = coe(3);
    O = coe(4);
    w = coe(5);
    M = coe(6);

    u = wrapToPi(w + M);
    ex = e * cos(w);
    ey = e * sin(w);

    qnsoe = [a; u; ex; ey; i; O];
end

