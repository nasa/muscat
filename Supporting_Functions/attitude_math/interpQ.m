function q_interp = interpQ(t_history, q_history, t_query)
%% INTERPQ: Interpolates quaternions to a new time vector using SLERP
    % t_history: Nx1 or 1xN vector of timestamps
    % q_history: Nx4 matrix (or 4xN, which will be transposed) of quaternions
    % t_query:   Mx1 or 1xM vector of new timestamps you want to interpolate onto
    
    %% --- Input Validation ---
    
    % Check that time inputs are vectors
    if ~isvector(t_history)
        error('Input Error: t_history must be a 1D vector.');
    end
    if ~isvector(t_query)
        error('Input Error: t_query must be a 1D vector.');
    end
    
    % Force time inputs to be column vectors to avoid dimension mismatch later
    t_history = t_history(:);
    t_query = t_query(:);
    
    % Check q_history dimensions and automatically transpose if necessary
    [num_history_pts, num_q_cols] = size(q_history);
    
    if num_q_cols ~= 4
        if num_history_pts == 4
            q_history = q_history';
            [num_history_pts, ~] = size(q_history); % Update row count
        else
            error('Input Error: q_history must have either 4 columns or 4 rows.');
        end
    end
    
    % Ensure the time history and quaternion history match in length
    if length(t_history) ~= num_history_pts
        error('Input Error: The length of t_history (%d) must match the number of elements in q_history (%d).', ...
              length(t_history), num_history_pts);
    end
    
    %% --- Interpolation Logic ---
    
    num_queries = length(t_query);
    q_interp = zeros(num_queries, 4); 
    
    for i = 1:num_queries
        t_req = t_query(i);
        
        % 1. Handle out-of-bounds (Assign NaNs)
        if t_req < t_history(1) || t_req > t_history(end)
            q_interp(i, :) = NaN(1, 4);
            continue;
            
        % 2. Catch the exact end boundary to prevent index out-of-bounds
        elseif t_req == t_history(end)
            q_interp(i, :) = q_history(end, :);
            continue;
        end
        
        % 3. Find the bounding indices in the history
        % (finds the last index where t_history is less than or equal to t_req)
        idx1 = find(t_history <= t_req, 1, 'last');
        idx2 = idx1 + 1;
        
        % 4. Extract the two bounding quaternions and their times
        q1 = q_history(idx1, :);
        q2 = q_history(idx2, :);
        t1 = t_history(idx1);
        t2 = t_history(idx2);
        
        % 5. Calculate the local interpolation parameter (normalized 0 to 1)
        local_t = (t_req - t1) / (t2 - t1);
        
        % 6. Call your original atomic SLERP function
        q_interp(i, :) = slerp(q1, q2, local_t);
    end
end