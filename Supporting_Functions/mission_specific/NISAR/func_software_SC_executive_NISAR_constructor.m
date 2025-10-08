%% [ ] Methods: NISAR Executive Constructor
% Constructor for Main Executive Function for NISAR mission

function obj = func_software_SC_executive_NISAR_constructor(obj, mission, i_SC)

i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;
[radius, lon, lat] = cspice_reclat(mission.true_target{i_target}.rotation_matrix' * mission.true_SC{i_SC}.true_SC_navigation.position_relative_target'); % [radius, longitude [rad], latitude [rad] ]
obj.data.longitude = rad2deg(lon); % [deg]
obj.data.latitude = rad2deg(lat); % [deg]
obj.data.altitude = radius - mission.true_target{i_target}.radius; % [km]


Sun_pos = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position; % [km]
Earth_pos = mission.true_target{i_target}.position; % [km]
SC_pos = mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
Sun_Earth_SC_Angle = func_angle_between_vectors( func_normalize_vec(Sun_pos - Earth_pos)', func_normalize_vec(SC_pos - Earth_pos)' ); % [rad]
obj.data.Sun_Earth_SC_Angle = rad2deg(Sun_Earth_SC_Angle); % [deg]

obj.data.prev_latitude = obj.data.latitude; % [deg]
obj.data.orbit_plane = 1; % [integer]

obj.data.this_side = 2; % [integer]
obj.data.flag_Recharge = 0; % [Boolean] This starts recharge mode
obj.data.SC_Target_Sun_angle = 0; 

obj.data.Telecom_GS_index = 0; % [integer]

% Initialize Variables to store:
obj.store.latitude = zeros(mission.storage.num_storage_steps, length(obj.data.latitude));
obj.store.longitude = zeros(mission.storage.num_storage_steps, length(obj.data.longitude));
obj.store.orbit_plane = zeros(mission.storage.num_storage_steps, length(obj.data.orbit_plane));
obj.store.this_side = zeros(mission.storage.num_storage_steps, length(obj.data.this_side));
obj.store.SC_Target_Sun_angle = zeros(mission.storage.num_storage_steps, length(obj.data.SC_Target_Sun_angle));
obj.store.flag_Recharge = zeros(mission.storage.num_storage_steps, length(obj.data.flag_Recharge));
obj.store.Telecom_GS_index = zeros(mission.storage.num_storage_steps, length(obj.data.Telecom_GS_index));
obj.store.Sun_Earth_SC_Angle = zeros(mission.storage.num_storage_steps, length(obj.data.Sun_Earth_SC_Angle)); 
