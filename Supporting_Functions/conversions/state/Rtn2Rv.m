function rv_d = Rtn2Rv(rv_c, rv_rtn)
rv_d = VecFcn(@Rtn2Rv_, rv_c, rv_rtn);
end

function rv_d = Rtn2Rv_(rv_c, rv_rtn)
% rv_c   6 x 1 [km]
% rv_d   6 x 1 [km]
% rv_rtn 6 x 1 [km]

r_c = rv_c(1:3);
v_c = rv_c(4:6);
Reci2rtn = RotationEci2Rtn(rv_c);


h = cross(r_c, v_c);
omega = h / norm(r_c) ^ 2;

pos_rel_eci = Reci2rtn' * rv_rtn(1:3);
vel_rel_eci = Reci2rtn' * rv_rtn(4:6) + cross(omega, pos_rel_eci);

r_d = r_c + pos_rel_eci;
v_d = v_c + vel_rel_eci;

rv_d = [r_d; v_d];
end