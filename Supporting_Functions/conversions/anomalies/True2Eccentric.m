function E = True2Eccentric(f, e)
% Convert true anomaly to mean anomaly
% both are in radians
if (e < 0) || (e > 1)
    warning('Invalid Eccentricity: has to satisfy 0<e<1')
    E = 0;
else
    E = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(f / 2));
end

end
