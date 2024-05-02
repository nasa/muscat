function roe = QnsOe2Roe(qnsoe_c, qnsoe_d)
    roe = VecFcn(@QnsOe2Roe_, qnsoe_c, qnsoe_d);
end

function roe = QnsOe2Roe_(qnsoe_c, qnsoe_d)
    a = qnsoe_c(1);
    u = qnsoe_c(2);
    ex = qnsoe_c(3);
    ey = qnsoe_c(4);
    i = qnsoe_c(5);
    O = qnsoe_c(6);

    a_d = qnsoe_d(1);
    u_d = qnsoe_d(2);
    ex_d = qnsoe_d(3);
    ey_d = qnsoe_d(4);
    i_d = qnsoe_d(5);
    O_d = qnsoe_d(6);

    da = (a_d - a) / a;
    dl = (u_d - u) + (O_d - O) * cos(i);
    dex = ex_d - ex;
    dey = ey_d - ey;
    dix = i_d - i;
    diy = (O_d - O) * sin(i);

    roe = [da; dl; dex; dey; dix; diy] * a;
end
