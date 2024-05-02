function M = True2Mean(f, e)
% Convert true anomaly to mean anomaly
% both are in radians
E = TrueToEccentric(f, e);
M = EccentricToMean(E, e);

end
