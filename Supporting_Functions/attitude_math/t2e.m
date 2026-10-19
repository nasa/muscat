function angles = t2e(t, sequence)
% Extract Euler angles from DCM (3x3xN) for any sequence.
% Assumes DCM maps via C = R_seq(3) * R_seq(2) * R_seq(1)

[r, c, N] = size(t);
if r ~= 3 || c ~= 3, error('DCM must be 3x3xN'); end

seq = lower(char(sequence));
seq = strrep(seq, 'x', '1'); seq = strrep(seq, 'y', '2'); seq = strrep(seq, 'z', '3');

i = str2double(seq(1)); 
j = str2double(seq(2)); 
k_seq = str2double(seq(3));

is_proper = (i == k_seq);
even_perms = [1 2 3; 2 3 1; 3 1 2];

T = @(row, col) reshape(t(row, col, :), [N, 1]);
angles = zeros(N, 3);

if is_proper
    % --- Proper Euler (Symmetric: i-j-i) ---
    k_alt = 6 - i - j; % The third unused axis
    eps_val = 1;
    if ~any(all(even_perms == [i, j, k_alt], 2)), eps_val = -1; end
    
    s2 = hypot(T(i, j), T(i, k_alt));
    angles(:,2) = atan2(s2, T(i, i));
    
    is_safe = (s2 > 1e-10);
    if any(is_safe)
        angles(is_safe, 1) = atan2(T(i, j), -eps_val * T(i, k_alt));
        angles(is_safe, 3) = atan2(T(j, i),  eps_val * T(k_alt, i));
    end
else
    % --- Tait-Bryan (Non-Symmetric: i-j-k) ---
    eps_val = 1;
    if ~any(all(even_perms == [i, j, k_seq], 2)), eps_val = -1; end
    
    angles(:,2) = atan2(eps_val * T(k_seq, i), hypot(T(k_seq, k_seq), T(k_seq, j)));
    
    c2 = cos(angles(:,2));
    is_safe = (abs(c2) > 1e-10);
    if any(is_safe)
        angles(is_safe, 1) = atan2(-eps_val * T(k_seq, j), T(k_seq, k_seq));
        angles(is_safe, 3) = atan2(-eps_val * T(j, i), T(i, i));
    end
end

% --- UNIVERSAL SINGULARITY FALLBACK ---
% Automatically handles Gimbal Lock robustly for all 12 sequences
if any(~is_safe)
    angles(~is_safe, 1) = 0; % Arbitrary assignment
    
    % Axis indices for the intermediate rotation (j)
    ja = mod(j, 3) + 1; 
    jb = mod(j + 1, 3) + 1;
    
    % Axis indices for the final rotation (k_seq)
    ka = mod(k_seq, 3) + 1; 
    kb = mod(k_seq + 1, 3) + 1;
    
    unsafe_idx = find(~is_safe)';
    for idx = unsafe_idx
        th = angles(idx, 2);
        
        % Construct R_j(theta_2)^T
        R2_T = eye(3);
        R2_T(ja, ja) = cos(th); R2_T(jb, jb) = cos(th);
        R2_T(ja, jb) = -sin(th); R2_T(jb, ja) = sin(th);
        
        % Multiply C * R_j^T to dynamically extract the pure 3rd rotation
        C_mat = t(:, :, idx);
        C_new_ab = C_mat(ka, :) * R2_T(:, kb);
        C_new_aa = C_mat(ka, :) * R2_T(:, ka);
        
        angles(idx, 3) = atan2(C_new_ab, C_new_aa);
    end
end
end