%% Postprocess NISAR Mission: Output to Excel

function func_post_process_NISAR_output_to_Excel(mission, i_SC)

Variable_Names = {'Time [sec]', 'Date Time Stamp [UTC]', 'Latitude [deg]', 'Longitude [deg]', 'Altitude [km]', 'Earth in View?', 'Sun in View?', 'Orbit Plane', ' ', ...
            'SC Mode', 'Ground Station', 'L-band SAR', 'S-band SAR', ' ', ...
            'Sun-Earth-SC Angle [deg]', 'SC Position [km] J2000-frame', 'Earth Position [km] J2000-frame', 'Sun Position [km] J2000-frame'}; 


cellData = [];

attitude_ratio = round((mission.storage.k_storage_attitude - 1)/(mission.storage.k_storage - 1));

for j=1:1:length(Variable_Names)

    for i=1:1:mission.storage.k_storage

        switch Variable_Names{j}

            case ' '
                cellData{i,j} = '     ';

            case 'Time [sec]'
                cellData{i,j} = mission.true_time.store.time(i);

            case 'Date Time Stamp [UTC]'
                cellData{i,j} = sec2cal(mission.true_time.store.date(i));

            case 'Longitude [deg]'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.longitude(i);

            case 'Latitude [deg]'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.latitude(i);

            case 'Altitude [km]'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.altitude(i);

            case 'Earth in View?'
                cellData{i,j} = mission.true_SC{i_SC}.true_SC_navigation.store.flag_visible_Earth(i);

            case 'Sun in View?'
                cellData{i,j} = mission.true_SC{i_SC}.true_SC_navigation.store.flag_visible_Sun(i);

            case 'Orbit Plane'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.orbit_plane(i);

            case 'SC Mode'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.sc_modes{mission.true_SC{i_SC}.software_SC_executive.store.this_sc_mode_value(i)};

            case 'Ground Station'
                cellData{i,j} = mission.true_SC{i_SC}.software_SC_executive.store.Telecom_GS_index(i);
            
            case 'L-band SAR'
                equipment = mission.true_SC{i_SC}.true_SC_remote_sensing{1};
                if strcmp(equipment.name, 'L-band SAR')
                    cellData{i,j} = equipment.store.flag_executive(i);
                else
                    error('Wrong value! 1')
                end

            case 'S-band SAR'
                equipment = mission.true_SC{i_SC}.true_SC_remote_sensing{2};
                if strcmp(equipment.name, 'S-band SAR')
                    cellData{i,j} = equipment.store.flag_executive(i);
                else
                    error('Wrong value! 2')
                end

            
            
                %             case 'Data Collect [kb]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_generated(i);
                %
                %             case 'Data Download [kb]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_data_handling.store.instantaneous_data_removed(i);
                %
                %             case 'Data Rate for Download [kbps]'
                %                 if mission.true_SC{i_SC}.true_SC_communication_link{1}.store.flag_executive(i) == 1
                %                     cellData{i,j} = mission.true_SC{i_SC}.true_SC_communication_link{1}.store.this_data_rate(i);
                %                 else
                %                     cellData{i,j} = 0;
                %                 end
                %
                %             case 'Data in Memory SC CDH [kb]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_onboard_memory{1}.store.instantaneous_capacity(i);
                %
                %             case 'Data in Memory ChirpChirp SSR (Interferogram Data) [kb]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_onboard_memory{2}.store.instantaneous_capacity(i);
                %
                %             case 'Data in Memory ChirpChirp SSR (Raw Data) [kb]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_onboard_memory{3}.store.instantaneous_capacity(i);
                %
                %             case 'Solar Panel Sun Angle [deg]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_solar_panel{1}.store.Sun_incidence_angle(i);
                %
                %             case 'Power Generated [W]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_generated(i);
                %
                %             case 'Power Consumed [W]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_power_consumed(i);
                %
                %             case 'Energy [W hr]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_energy(i);
                %
                %             case 'Unused Energy [W hr]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_power.store.instantaneous_energy_unused(i);
                %
                %             case 'Battery Capacity [W hr]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_battery{1}.store.instantaneous_capacity(i);
                %
                %             case 'Battery SoC [percentage]'
                %                 cellData{i,j} = mission.true_SC{i_SC}.true_SC_battery{1}.store.state_of_charge(i);

            

            case 'SC Position [km] J2000-frame'                
                cellData{i,j} = mission.true_SC{i_SC}.true_SC_navigation.store.position(i,:); % [km]

            case 'Earth Position [km] J2000-frame'
                cellData{i,j} = mission.true_target{1}.store.position(i,:); % [km]

            case 'Sun Position [km] J2000-frame'
                cellData{i,j} = mission.true_solar_system.store.SS_body{mission.true_solar_system.index_Sun}.position(i,:); % [km]

            case 'Sun-Earth-SC Angle [deg]'
                Sun_pos = mission.true_solar_system.store.SS_body{mission.true_solar_system.index_Sun}.position(i,:); % [km]
                Earth_pos = mission.true_target{1}.store.position(i,:); % [km]
                SC_pos = mission.true_SC{i_SC}.true_SC_navigation.store.position(i,:); % [km]

                Sun_Earth_SC_Angle = func_angle_between_vectors( func_normalize_vec(Sun_pos - Earth_pos)', func_normalize_vec(SC_pos - Earth_pos)' ); % [rad]
                cellData{i,j} = rad2deg(Sun_Earth_SC_Angle); %[deg]

            otherwise
                error('Should not reach here!')

        end

    end

end


% Combine headers and data into a single cell array
dataWithHeaders = [Variable_Names; cellData];

% Write cell array to Excel file
writecell(dataWithHeaders, [mission.storage.output_folder, mission.name,' ',num2str(i_SC),'SC output_with_headers.xlsx']);
