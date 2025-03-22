%% [ ] Methods: Constructor for DART Executive

function obj = func_software_SC_executive_Dart_constructor(obj, mission,i_SC)
% Add mode transition management
if ~isfield(obj.data, 'last_mode_change_time')
    obj.data.last_mode_change_time = 0;
    obj.data.previous_mode = obj.this_sc_mode;
end
end
