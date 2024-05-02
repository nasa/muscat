function M = Eccentric2Mean(E, e)
% Convert true anomaly to mean anomaly
% both are in radians
if (e < 0)
    warning('Invalid Eccentricity: has to satisfy 0<e')
    E = 0;
else
    M = wrapToPi(E - e * sin(E));
end

end
