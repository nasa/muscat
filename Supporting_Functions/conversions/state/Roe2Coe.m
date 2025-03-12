function oe_d = Roe2Coe(oe_c, roe)
oe_d = VecFcn(@Roe2Coe_,oe_c, roe);
end

function oe_d = Roe2Coe_(oe_c, roe)
% oe  : Classical OE
%       (6 x 1)
%       [a,  e,   i,   O,   w,   M]'
%       [km, -, rad, rad, rad, rad]'

% roe : Quasi-Nonsingular ROE
%       (6 x 1)
%       [ada, adl_e, ade_x, ade_y, adi_x, adi_y]'
%       [ km,    km,    km,    km,    km,    km]'

a_c = oe_c(1);
e_c = oe_c(2);
i_c = oe_c(3);
O_c = oe_c(4);
w_c = oe_c(5);
M_c = oe_c(6);

da   = roe(1) / a_c;
dl   = roe(2) / a_c;
de_x = roe(3) / a_c;
de_y = roe(4) / a_c;
di_x = roe(5) / a_c;
di_y = roe(6) / a_c;

u_c  = wrapToPi(w_c + M_c);
e_xc = e_c * cos(w_c);
e_yc = e_c * sin(w_c);
dO   = wrapToPi(di_y / sin(i_c));
du   = wrapToPi(dl - (dO * cos(i_c)));
e_xd = e_xc + de_x;
e_yd = e_yc + de_y;
u_d  = wrapToPi(u_c + du);

a_d = a_c + da * a_c;
e_d = sqrt(e_xd ^ 2 + e_yd ^ 2);
i_d = wrapToPi(i_c + di_x);
O_d = wrapToPi(O_c + dO);
w_d = wrapToPi(atan2(e_yd, e_xd));
M_d = wrapToPi(u_d - w_d);

oe_d = [a_d, e_d, i_d, O_d, w_d, M_d]';
end