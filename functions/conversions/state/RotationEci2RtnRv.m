function Rot = RotationEci2RtnRv(rv_c)

r_c = rv_c(1:3);
v_c = rv_c(4:6);
Reci2rtn = RotationEci2Rtn(rv_c);
h = cross(r_c, v_c);
omega = h / norm(r_c) ^ 2;

Rot = [
    Reci2rtn,    zeros(3);
    skew(omega), Reci2rtn];
end