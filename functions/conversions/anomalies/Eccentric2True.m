function f = Eccentric2True(E, e)
f = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2));
end
