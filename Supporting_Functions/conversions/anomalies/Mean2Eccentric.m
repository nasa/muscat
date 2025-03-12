function E = Mean2Eccentric(M, e)
E = fzero(@(x) Kepler(x, M, e), M); % solve Kepler Equation
end

function diff = Kepler(E, M, e)
diff = E - e * sin(E) - M;
end
