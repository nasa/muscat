function rv_rtn = Rv2Rtn(rv_c, rv_d)
rv_rtn = VecFcn(@Rv2Rtn_, rv_c, rv_d);
end

function rv_rtn = Rv2Rtn_(rv_c, rv_d)
r_c = rv_c(1:3);
v_c = rv_c(4:6);
Reci2rtn = RotationEci2Rtn(rv_c);

r_d = rv_d(1:3);
v_d = rv_d(4:6);

pos_rel_eci = r_d - r_c;
vel_rel_eci = v_d - v_c;

h = cross(r_c, v_c);
omega = h / norm(r_c) ^ 2;

rv_rtn = zeros(6,1);
rv_rtn(1:3) = Reci2rtn * pos_rel_eci;
rv_rtn(4:6) = Reci2rtn * (vel_rel_eci - cross(omega, pos_rel_eci));
end