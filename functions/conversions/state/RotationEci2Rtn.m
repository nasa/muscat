function T = RotationEci2Rtn(X)
% ECI2RTN Return transformation matrix for ECI to RTN
% Provided some reference orbit, rstar and vstar, the conversion matrix
% between the radial, transverse, and normal (radial, along-track,
% cross-track) and earth centered inertial frame is returned.

rstar = reshape(X(1:3), 3, 1); %position vector, meters
vstar = reshape(X(4:6), 3, 1); %velocity vector, meters/second

ur = rstar / norm(rstar);
un = cross(rstar, vstar) / norm(cross(rstar, vstar));
ut = cross(un, ur);

T = [ur';ut';un'];

end
