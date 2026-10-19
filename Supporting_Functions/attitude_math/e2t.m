function t = e2t(angles, sequence)
% Converts Euler angles (Nx3) to DCM (3x3xN) for any sequence.
% Constructs C = R_seq(3) * R_seq(2) * R_seq(1)

[N, cols] = size(angles);
if cols ~= 3, error('Angles must be Nx3'); end

seq = lower(char(sequence)); 
seq = strrep(seq, 'x', '1'); seq = strrep(seq, 'y', '2'); seq = strrep(seq, 'z', '3');

% Start with an Identity DCM for every sample
t = repmat(eye(3), [1, 1, N]);

% Apply three rotations sequentially
for i = 1:3
    axis_idx = str2double(seq(i)); 
    theta = angles(:,i);
    
    Ri = repmat(eye(3), [1, 1, N]);
    c = cos(theta); s = sin(theta);
    
    % Correctly mapped indices for rotation sub-matrix (passive frame rotation)
    a = mod(axis_idx, 3) + 1;
    b = mod(axis_idx + 1, 3) + 1;
    
    Ri(a, a, :) = reshape(c, [1, 1, N]);
    Ri(b, b, :) = reshape(c, [1, 1, N]);
    Ri(a, b, :) = reshape(s, [1, 1, N]);
    Ri(b, a, :) = reshape(-s, [1, 1, N]);
    
    if exist('pagemtimes', 'builtin')
        t = pagemtimes(Ri, t);
    else
        % Safe fallback without dimension mismatch errors
        t_new = zeros(3, 3, N);
        for row = 1:3
            for col = 1:3
                % Proper dot product over dimension 2
                t_new(row, col, :) = sum(Ri(row, 1:3, :) .* reshape(t(1:3, col, :), [1, 3, N]), 2);
            end
        end
        t = t_new;
    end
end
end