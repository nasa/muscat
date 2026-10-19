%% [ ] Methods: Initialize RCS Thruster Contribution Matrix

function obj = func_initialize_RCS_thruster_contribution(obj, mission, i_SC)
% Build thruster torque contribution matrix
num_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_RCS_thruster;
obj.thruster_contribution_matix = zeros(3, num_thrusters);
obj.max_thrust = zeros(1, num_thrusters);
obj.min_thrust = zeros(1, num_thrusters);

for i = 1:num_thrusters
    thruster = mission.true_SC{i_SC}.true_SC_RCS_thruster{i};
    r = thruster.location - mission.true_SC{i_SC}.true_SC_body.location_COM;
    obj.thruster_contribution_matix(:,i) = cross(r, thruster.orientation);
    obj.max_thrust(i) = thruster.maximum_thrust;
    obj.min_thrust(i) = thruster.minimum_thrust;
end
end
