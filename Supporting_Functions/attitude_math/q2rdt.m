function [ra, dec, tw] = q2rdt(q)
% function [ra,dec,tw] = q2rdt(q);
% Vectorized conversion from quaternion to Celestial Euler angles.
% Sequence: Z(ra) -> Y(pi/2 - dec) -> Z(twist)

[m, n] = size(q);
is_transposed = false;

% Normalize to N x 4
if n ~= 4
    if m == 4
        q = q.';
        [m, n] = size(q);
        is_transposed = true;
    else
        error('[q2rdt] Input must be (N,4) or (4,N)');
    end
end

% 1. Extract quaternion components
q1 = q(:,1); q2 = q(:,2); q3 = q(:,3); q4 = q(:,4);

% 2. Extract necessary DCM elements for the Z-Y-Z sequence
% t31, t32, t33 are the primary components for RA and Dec
t31 = 2 * (q1.*q3 + q4.*q2);
t32 = 2 * (q2.*q3 - q1.*q4);
t33 = q4.^2 + q3.^2 - (q1.^2 + q2.^2);

z12 = hypot(t31, t32);

% Preallocate outputs
ra  = zeros(m, 1);
dec = zeros(m, 1);
tw  = zeros(m, 1);

% 3. Handle Singularities (Polar Pointing)
% The original code used: t33 == t33 + z12
is_polar = (z12 == 0); 
not_polar = ~is_polar;

% Case A: Polar Pointing
if any(is_polar)
    % At the poles, we define ra = 0 and calculate tw from t12 and t22
    t12_p = 2 * (q1(is_polar).*q2(is_polar) + q4(is_polar).*q3(is_polar));
    t22_p = q4(is_polar).^2 + q2(is_polar).^2 - (q3(is_polar).^2 + q1(is_polar).^2);
    
    ra(is_polar)  = 0.0;
    dec(is_polar) = (pi/2) .* sign(t33(is_polar));
    tw(is_polar)  = atan2(t12_p, t22_p);
end

% Case B: Standard pointing
if any(not_polar)
    t23_s = 2 * (q2(not_polar).*q3(not_polar) + q1(not_polar).*q4(not_polar));
    t13_s = 2 * (q1(not_polar).*q3(not_polar) - q4(not_polar).*q2(not_polar));
    
    ra(not_polar)  = atan2(t32(not_polar), t31(not_polar));
    dec(not_polar) = atan2(t33(not_polar), z12(not_polar)); % atan(y/x) -> atan2(y,x)
    tw(not_polar)  = atan2(t23_s, -t13_s);
end

% Match input orientation
if is_transposed
    ra = ra.'; dec = dec.'; tw = tw.';
end

end