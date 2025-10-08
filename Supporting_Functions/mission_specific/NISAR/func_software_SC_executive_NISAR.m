%% [ ] Methods: NISAR Executive
% Main Executive Function for NISAR mission

function obj = func_software_SC_executive_NISAR(obj, mission, i_SC)

%% Switch on standard softwares
mission.true_SC{i_SC}.software_SC_estimate_attitude.flag_executive = 1;
mission.true_SC{i_SC}.software_SC_estimate_orbit.flag_executive = 1;
mission.true_SC{i_SC}.software_SC_control_attitude.flag_executive = 1;
mission.true_SC{i_SC}.software_SC_power.flag_executive = 1;

%% Compute Latitude and Longitude

obj.data.prev_latitude = obj.data.latitude; % [deg]

i_target = mission.true_SC{i_SC}.true_SC_navigation.index_relative_target;

[radius, lon, lat] = cspice_reclat(mission.true_target{i_target}.rotation_matrix' * mission.true_SC{i_SC}.true_SC_navigation.position_relative_target'); % [radius, longitude [rad], latitude [rad] ]
obj.data.longitude = rad2deg(lon); % [deg]
obj.data.latitude = rad2deg(lat); % [deg]
obj.data.altitude = radius - mission.true_target{i_target}.radius; % [km]

% Update Orbit Plane number
if (obj.data.prev_latitude <= 0) && (obj.data.latitude >= 0)
    obj.data.orbit_plane = obj.data.orbit_plane + 1;
end

% Update Lit or Dark Side (1 = Dark side, 2 = Lit side)
obj.data.SC_Target_Sun_angle = func_angle_between_vectors([mission.true_SC{i_SC}.software_SC_estimate_orbit.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target]', [mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position - mission.true_SC{i_SC}.software_SC_estimate_orbit.position_target]' ); % [rand]
if abs(obj.data.SC_Target_Sun_angle) <= pi/2
    obj.data.this_side = 2; % Lit Side
else
    obj.data.this_side = 1; % Dark Side
end

Sun_pos = mission.true_solar_system.SS_body{mission.true_solar_system.index_Sun}.position; % [km]
Earth_pos = mission.true_target{i_target}.position; % [km]
SC_pos = mission.true_SC{i_SC}.true_SC_navigation.position; % [km]
Sun_Earth_SC_Angle = func_angle_between_vectors( func_normalize_vec(Sun_pos - Earth_pos)', func_normalize_vec(SC_pos - Earth_pos)' ); % [rad]
obj.data.Sun_Earth_SC_Angle = rad2deg(Sun_Earth_SC_Angle); % [deg]

%% Executive logic for NISAR mission

% Check Telecom
obj.data.Telecom_GS_index = 0;
for i_HW = 1:1:mission.true_ground_station.num_GS_radio_antenna

    this_GS_position = (mission.true_target{i_target}.rotation_matrix * mission.true_GS_radio_antenna{i_HW}.position')'; % [km]
    this_SC_position = mission.true_SC{i_SC}.true_SC_navigation.position_relative_target; % [km]
    elevation_angle = rad2deg( func_angle_between_vectors( func_normalize_vec(this_GS_position), func_normalize_vec (this_SC_position - this_GS_position) ) ); 

    if elevation_angle <= 80 % [deg] threshold
        obj.data.Telecom_GS_index = i_HW;
    end

end

if obj.data.Telecom_GS_index > 0

    obj.this_sc_mode = 'Telecom';
    mission.true_SC{i_SC}.software_SC_communication.flag_executive = 1;

    if ((obj.data.latitude < -91) || (obj.data.latitude > 91)) && (mod(obj.data.orbit_plane, 5)~=0)

        % No Science

    else

        % Do Science too!

        % Switch on L-band radar
        mission.true_SC{i_SC}.true_SC_remote_sensing{1}.flag_executive = 1;

        % When SC is over India, swith on S-band radar
        % India spans roughly from 8°4′N to 37°6′N latitude and 68°7′E to 97°25′E longitude.

        if (obj.data.latitude > 8) && (obj.data.latitude < 38) && (obj.data.longitude > 68) && (obj.data.longitude < 98)

            % Switch on S-band radar
            mission.true_SC{i_SC}.true_SC_remote_sensing{2}.flag_executive = 1;

        end

    end


elseif ((obj.data.latitude < -91) || (obj.data.latitude > 91)) && (mod(obj.data.orbit_plane, 5)~=0)

    obj.this_sc_mode = 'Maximize SP Power';    

else

    obj.this_sc_mode = 'Science Mode';

    % Switch on L-band radar
    mission.true_SC{i_SC}.true_SC_remote_sensing{1}.flag_executive = 1;

    % When SC is over India, swith on S-band radar
    % India spans roughly from 8°4′N to 37°6′N latitude and 68°7′E to 97°25′E longitude.

    if (obj.data.latitude > 8) && (obj.data.latitude < 38) && (obj.data.longitude > 68) && (obj.data.longitude < 98)

        % Switch on S-band radar
        mission.true_SC{i_SC}.true_SC_remote_sensing{2}.flag_executive = 1;

    end

end
