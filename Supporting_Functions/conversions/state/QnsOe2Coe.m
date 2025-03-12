function qnsoe = QnsOe2Coe(coe)
    qnsoe = VecFcn(@QnsOe2Coe_, coe);
end
    
function coe = QnsOe2Coe_(qnsoe)
    a = qnsoe(1);
    u = qnsoe(2);
    ex = qnsoe(3);
    ey = qnsoe(4);
    i = qnsoe(5);
    O = qnsoe(6);

    e = sqrt(ex ^ 2 + ey ^ 2);
    w = wrapToPi(atan2(ey, ex));
    M = wrapToPi(u - w);

    coe = [a, e, i, O, w, M]';
end
