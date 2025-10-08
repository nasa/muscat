%% [ ] Methods: Store for NISAR
% Update the store variable

function obj = func_update_software_SC_executive_store_NISAR(obj, mission)

obj.store.latitude(mission.storage.k_storage,:) = obj.data.latitude;
obj.store.longitude(mission.storage.k_storage,:) = obj.data.longitude;
obj.store.altitude(mission.storage.k_storage,:) = obj.data.altitude;
obj.store.Sun_Earth_SC_Angle(mission.storage.k_storage,:) = obj.data.Sun_Earth_SC_Angle;

obj.store.orbit_plane(mission.storage.k_storage,:) = obj.data.orbit_plane;
obj.store.this_side(mission.storage.k_storage,:) = obj.data.this_side;
obj.store.SC_Target_Sun_angle(mission.storage.k_storage,:) = obj.data.SC_Target_Sun_angle;
obj.store.flag_Recharge(mission.storage.k_storage,:) = obj.data.flag_Recharge;
obj.store.Telecom_GS_index(mission.storage.k_storage,:) = obj.data.Telecom_GS_index;

