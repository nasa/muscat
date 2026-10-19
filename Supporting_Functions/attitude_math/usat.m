function [xsat] = usat(x, bound)
% function [xsat] = usat(x,bound);
% Returns a saturated version of the input x, clamped within [-bound, bound].
% x can be a scalar, vector, or matrix.
% bound is typically a scalar, but can be a matrix matching size(x).

xsat = min(max(x, -bound), bound);

end