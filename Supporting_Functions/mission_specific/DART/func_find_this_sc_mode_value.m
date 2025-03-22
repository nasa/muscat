%% [ ] Methods: Find SC Mode Value
% Finds the value of a given SC mode in Executive

function val = func_find_this_sc_mode_value(obj, this_sc_mode)
% Find the index of the given SC mode
IndexC = strcmp(obj.sc_modes, this_sc_mode);
val = find(IndexC);

% Check if the result is empty
if isempty(val)
    % Raise an error if nothing is found
    error('SC mode "%s" not found in obj.sc_modes.', this_sc_mode);
end
end
