%% [ ] Methods: Initialize Reaction Wheel Contribution Matrix for Pinverse Optimization

function obj = func_initialize_reaction_wheel_contribution(obj, mission, i_SC)
% Build RWA momentum contribution matrix
reaction_wheel_contribution_matrix = zeros(3,  mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel);

for i = 1: mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_reaction_wheel
    wheel = mission.true_SC{i_SC}.true_SC_reaction_wheel{i};
    if wheel.health
        reaction_wheel_contribution_matrix(:,i) = wheel.orientation' * wheel.moment_of_inertia;
    end
end

obj.pinv_reaction_wheel_contribution_matrix = pinv(reaction_wheel_contribution_matrix);
end
