function mx = getMaxValueAboutAxis(lam,maxVal)
% Used for computing ellipsoidal limits for an input axis
%  axis - input axis for which to find the ellipsoidal limit
%  maxVal - Maximum value along each axis of the ellipsoid
mx = 1/vecnorm(lam./maxVal,2,2);