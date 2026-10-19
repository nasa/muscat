%% [ ] Methods: Additional Storage for Control Orbit for DART

function obj = func_update_sw_SC_ctrl_orbit_store_DART(obj, mission)

obj.store.intercept_distance(mission.storage.k_storage,:) = obj.data.intercept_distance; 
obj.store.desired_DeltaV_computed(mission.storage.k_storage,:) = obj.data.desired_DeltaV_computed;
obj.store.desired_attitude_for_DeltaV_achieved(mission.storage.k_storage,:) = obj.data.desired_attitude_for_DeltaV_achieved;

% Delta-V data
obj.store.desired_control_DeltaV(mission.storage.k_storage,:) = obj.data.desired_control_DeltaV;
obj.store.total_DeltaV_executed(mission.storage.k_storage,:) = obj.data.total_DeltaV_executed;

% Fuel management data
obj.store.flag_insufficient_fuel(mission.storage.k_storage,:) = obj.data.flag_insufficient_fuel;
obj.store.estimated_fuel_required(mission.storage.k_storage,:) = obj.data.estimated_fuel_required;


end