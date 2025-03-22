%% [ ] Methods: Update torque capabilities
% Should be called if hardware configuration/health changes

function obj = func_update_torque_capabilities(obj, mission, i_SC)
% Recalculate maximum torque capabilities after hardware changes
obj.max_rw_torque = func_calculate_max_reaction_wheel_torque(obj, mission, i_SC);
obj.max_mt_torque = func_calculate_max_thruster_torque(obj, mission, i_SC);
end

