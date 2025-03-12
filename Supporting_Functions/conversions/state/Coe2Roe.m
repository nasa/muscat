function roe = Coe2Roe(oe_c, oe_d)
roe = VecFcn(@Coe2Roe_, oe_c, oe_d);
end

function roe = Coe2Roe_(oe_c, oe_d)
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

a_d = oe_d(1);
e_d = oe_d(2);
i_d = oe_d(3);
O_d = oe_d(4);
w_d = oe_d(5);
M_d = oe_d(6);

dM = wrapToPi(M_d - M_c);
dw = wrapToPi(w_d - w_c);
dO = wrapToPi(O_d - O_c);

da   = (a_d - a_c) / a_c;
dl_e = dM + dw + dO * cos(i_c);
de_x = e_d*cos(w_d) - e_c*cos(w_c);
de_y = e_d*sin(w_d) - e_c*sin(w_c);
di_x = wrapToPi(i_d - i_c);
di_y = wrapToPi(O_d - O_c)*sin(i_c);

roe = a_c * [da, dl_e, de_x, de_y, di_x, di_y]';
end
