function [v] = rade2v(ra, dec)
% function [v] = rade2v(ra,dec)
% Transform Right Ascension and Declination to a Cartesian unit vector.
% Handles (N,1) vectors or (1,N) vectors. Returns (N,3).

% Ensure inputs are column vectors for consistent concatenation
ra = ra(:);
dec = dec(:);

if length(ra) ~= length(dec)
    error('[rade2v] Right Ascension and Declination must be the same length.');
end

% Precompute cos(dec) to save redundant transcendental calls
cos_dec = cos(dec);

% Generate Cartesian components:
% x = cos(dec) * cos(ra)
% y = cos(dec) * sin(ra)
% z = sin(dec)
v = [cos_dec .* cos(ra), cos_dec .* sin(ra), sin(dec)];

end