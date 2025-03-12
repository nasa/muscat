function f = Mean2True(M, e)

E = Mean2Eccentric(M, e);
f = Eccentric2True(E, e);

end
