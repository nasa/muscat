%% [ ] Methods: Checks if desaturation is needed

function needed = func_is_desaturation_needed(obj, mission, i_SC)
% Simplified function to check if wheel desaturation is needed
% Returns true if any wheel is saturated or desaturation is in progress

needed = false;

% If desaturation is already in progress, continue it
if obj.desaturation_procedure
    needed = true;
    return;
end

% Check each reaction wheel for saturation
for i = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel

    % Only consider healthy wheels
    if mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.health && mission.true_SC{i_SC}.true_SC_reaction_wheel{i}.saturated
        needed = true;
        return;
    end
end
end
