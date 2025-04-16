function func_plot_orbital_control_performance(mission, i_SC)
    % Make sure kd is valid and within range
    kd = min(mission.storage.k_storage, size(mission.true_time.store.time, 1));
    
    % Ensure we have at least one data point
    if kd < 1
        warning('No valid data points available for plotting');
        return;
    end
    
    % Check hardware/software existence
    has_chemical_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster > 0;
    has_micro_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster > 0;
    has_orbit_control = isfield(mission.true_SC{i_SC}, 'software_SC_control_orbit');
    has_fuel_tanks = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 0;
    
    % Use a 3x3 layout
    nb_row = 3;
    nb_col = 3;
    
    %% Figure setup
    plot_handle = figure('Name',['SC ', num2str(i_SC), ' Orbital Control Performance']);
    clf;
    set(plot_handle, 'Color', [1, 1, 1]);
    set(plot_handle, 'units', 'normalized', 'outerposition', [0, 0, 1, 1]);
    set(plot_handle, 'PaperPositionMode', 'auto');
    colors = {'r', 'g', 'b', 'c', 'm', 'y', 'k', [0.5, 0.5, 0.5]};
    
    %% Row 1: Chemical Thruster Info
    if has_chemical_thrusters
        num_thrusters = mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster;
        
        % Only use the first thruster for plots (typically only one chemical thruster)
        i_CT = 1;
        thruster = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_CT};
        
        % Plot 1: Thrust plot
        subplot(nb_row, nb_col, 1);
        hold on;
        plot(mission.true_time.store.time(1:kd), thruster.store.commanded_thrust(1:kd), '-r', 'LineWidth', 1.5, 'DisplayName', 'Commanded');
        plot(mission.true_time.store.time(1:kd), thruster.store.true_commanded_thrust(1:kd), '--b', 'LineWidth', 1.5, 'DisplayName', 'True');
        plot(mission.true_time.store.time(1:kd), ones(kd,1)*thruster.maximum_thrust, ':k', 'HandleVisibility', 'off');
        grid on; xlabel('Time [sec]'); ylabel('Thrust [N]');
        title([thruster.name ' Thrust']); legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;

        % Plot 2: Force vector components plot
        subplot(nb_row, nb_col, 2);
        hold on;
        
        % Get force data
        if isfield(thruster.store, 'force_inertial')
            try
                force_data = thruster.store.force_inertial(1:kd, :);
                
                % Add debug information about the data
                if any(any(isnan(force_data))) || any(any(isinf(force_data)))
                    warning('Force data contains NaN or Inf values');
                end
                
                % Display summary of force data
                max_force = max(max(abs(force_data)));
                disp(['Max force magnitude: ' num2str(max_force) ' N']);
                
                % Check if force data has any significant values
                if max_force < 1e-6
                    text(mean(mission.true_time.store.time(1:kd)), 0, 'Force values near zero', 'FontSize', 12, 'HorizontalAlignment', 'center');
                else
                    % Ensure we're plotting data even if values are small
                    for i = 1:3
                        plot(mission.true_time.store.time(1:kd), force_data(:, i), '-', 'LineWidth', 1.5, 'Color', colors{i}, 'DisplayName', ['F' char('X'+i-1)]);
                        
                        % Add annotation for peak values for better visibility
                        [max_val, max_idx] = max(abs(force_data(:, i)));
                        if max_val > 0.001 % Only annotate if there's a significant value
                            text(mission.true_time.store.time(max_idx), force_data(max_idx, i), ...
                                 [' ' num2str(force_data(max_idx, i), '%.3f') ' N'], ...
                                 'FontSize', 8, 'Color', colors{i});
                        end
                    end
                end
            catch e
                % Handle any errors in force data processing
                warning(['Error processing force data: ' e.message]);
                text(0.5, 0.5, 'Error processing force data', 'FontSize', 12, 'HorizontalAlignment', 'center');
            end
        else
            % Handle case where force_inertial doesn't exist
            text(0.5, 0.5, 'No force data available', 'FontSize', 12, 'HorizontalAlignment', 'center');
        end
        
        grid on; xlabel('Time [sec]'); ylabel('Force [N]');
        title([thruster.name ' Inertial Force']); legend('Location', 'southeast');
        ylim([-1, 1]); % Set a reasonable y-limit for better visibility
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
        
        % Plot 3: Thruster State Machine
        subplot(nb_row, nb_col, 3);
        hold on;
        
        % Only proceed if thruster state is available
        if isfield(thruster.store, 'thruster_state')
            % Define state bands with colors
            states = {'idle', 'warming up', 'ready', 'firing'};
            state_colors = {[0.8, 0.8, 0.8], [1, 0.8, 0.8], [0.8, 1, 0.8], [0.8, 0.8, 1]};
            
            % Draw colored bands for each state
            for j = 1:length(states)
                y_pos = j;
                fill([mission.true_time.store.time(1) mission.true_time.store.time(kd) mission.true_time.store.time(kd) mission.true_time.store.time(1)], ...
                     [y_pos-0.25 y_pos-0.25 y_pos+0.25 y_pos+0.25], ...
                     state_colors{j}, 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
                     'DisplayName', states{j});
            end
            
            % Convert thruster states to numerical values but in a memory-efficient way
            % For large kd, process in chunks to avoid excessive memory usage
            chunk_size = 1000;
            num_chunks = ceil(kd / chunk_size);
            
            % Instead of creating a full array, plot in chunks
            last_state = 1; % Default to idle
            state_changes = [];
            change_times = [];
            
            for chunk = 1:num_chunks
                start_idx = (chunk-1) * chunk_size + 1;
                end_idx = min(chunk * chunk_size, kd);
                
                for i = start_idx:end_idx
                    current_state = 1; % Default to idle
                    
                    if i <= length(thruster.store.thruster_state) && ...
                       ~isempty(thruster.store.thruster_state{i}) && ...
                       ischar(thruster.store.thruster_state{i})
                        switch thruster.store.thruster_state{i}
                            case 'idle'
                                current_state = 1;
                            case 'warming_up'
                                current_state = 2;
                            case 'ready'
                                current_state = 3;
                            case 'firing'
                                current_state = 4;
                        end
                    end
                    
                    % Record state changes only
                    if current_state ~= last_state
                        state_changes = [state_changes; last_state; current_state];
                        change_times = [change_times; mission.true_time.store.time(max(i-1,1)); mission.true_time.store.time(i)];
                        last_state = current_state;
                    end
                end
            end
            
            % Add the final state to close the plot
            if ~isempty(state_changes)
                state_changes = [state_changes; last_state];
                change_times = [change_times; mission.true_time.store.time(kd)];
                
                % Plot the state changes as line segments
                plot(change_times, state_changes, '-k', 'LineWidth', 2, 'DisplayName', 'Thruster State');
                
                % Mark state transitions with circles
                for i = 2:2:length(change_times)-1
                    plot(change_times(i), state_changes(i), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
                end
            else
                % No state changes, just plot a flat line at the default state
                plot([mission.true_time.store.time(1) mission.true_time.store.time(kd)], [1 1], '-k', 'LineWidth', 2, 'DisplayName', 'Thruster State');
            end
            
            % Add labels for state values
            yticks(1:4);
            yticklabels(states);
            ylim([0.5, 4.5]);
            
            % Add commanded thrust information but don't use yyright axis
            % Instead, scale and offset the thrust data to overlay it
            max_thrust = max(thruster.store.commanded_thrust(1:kd));
            if max_thrust > 0
                scaled_thrust = 0.5 + 3.5 * (thruster.store.commanded_thrust(1:kd) / max_thrust);
                plot(mission.true_time.store.time(1:kd), scaled_thrust, '--r', 'LineWidth', 1, 'DisplayName', 'Scaled Thrust');
            end
        else
            text(0.5, 0.5, 'No thruster state data available', 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
        
        % Set titles and labels
        grid on;
        title('Thruster State');
        xlabel('Time [sec]');
        ylabel('State');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
    else
        % If no chemical thrusters, display message in first row plots
        for i = 1:3
            subplot(nb_row, nb_col, i);
            text(0.5, 0.5, 'No chemical thrusters present', 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
            title(['Plot ' num2str(i)]);
        end
    end
    
    %% Row 2: Orbit Control and DeltaV
    if has_orbit_control
        orbit_control = mission.true_SC{i_SC}.software_SC_control_orbit;
        time_vec = mission.true_time.store.time(1:kd);
        
        % Plot 4: DeltaV Combined Plot
        subplot(nb_row, nb_col, 4);
        hold on;
        
        % Add thruster alignment mode at the top
        if isfield(mission.true_SC{i_SC}, 'software_SC_executive')
            mode_values = mission.true_SC{i_SC}.software_SC_executive.store.this_sc_mode_value(1:mission.storage.k_storage);
            is_deltaV_mode = strcmp(mission.true_SC{i_SC}.software_SC_executive.sc_modes(mode_values), 'Point Thruster along DeltaV direction');
            
            % Plot mode as shaded area
            area(time_vec, double(is_deltaV_mode)*max(orbit_control.store.deltaV_magnitude_desired(1:kd))*1.1, ...
                'FaceColor', [0.9, 0.9, 1], 'EdgeColor', 'none', 'DisplayName', 'Alignment Mode');
        end
        
        % Plot DeltaV Magnitude
        plot(time_vec, orbit_control.store.deltaV_magnitude_desired(1:kd), '-r', 'LineWidth', 2, 'DisplayName', 'Desired ΔV');
        plot(time_vec, orbit_control.store.deltaV_magnitude_executed(1:kd), '--b', 'LineWidth', 2, 'DisplayName', 'Executed ΔV');
        
        % Mark maneuver start points
        if isfield(orbit_control, 'time_DeltaV')
            for i = 2:kd
                % Look for moments when desired DeltaV changed significantly
                if orbit_control.store.deltaV_magnitude_desired(i) > 0 && orbit_control.store.deltaV_magnitude_desired(i-1) == 0
                    plot(time_vec(i), orbit_control.store.deltaV_magnitude_desired(i), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Maneuver Start');
                end
            end
        end
        
        % Mark completed maneuvers
        for i = 2:kd
            if orbit_control.store.deltaV_magnitude_desired(i) == 0 && orbit_control.store.deltaV_magnitude_desired(i-1) > 0
                plot(time_vec(i-1), orbit_control.store.deltaV_magnitude_executed(i-1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Maneuver Complete');
            end
        end
        
        grid on; xlabel('Time [sec]'); ylabel('ΔV Magnitude [m/s]');
        title('ΔV Progress and Thruster Alignment'); 
        legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
        
        % Plot 5: DeltaV Components
        subplot(nb_row, nb_col, 5);
        hold on;
        for i = 1:3
            plot(time_vec, orbit_control.store.desired_control_DeltaV(1:kd, i), '-', 'LineWidth', 1.5, 'Color', colors{i}, 'DisplayName', ['ΔV' char('X'+i-1)]);
            plot(time_vec, orbit_control.store.total_DeltaV_executed(1:kd, i), '--', 'LineWidth', 1.5, 'Color', colors{i}, 'DisplayName', ['Executed ' char('X'+i-1)]);
        end
        grid on; xlabel('Time [sec]'); ylabel('ΔV [m/s]');
        title('Desired vs. Executed ΔV Components'); legend('Location', 'southeast');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
        
        % Plot 6: Empty (removed additional orbit control info)
        subplot(nb_row, nb_col, 6);
        hold off;
        set(gca, 'Visible', 'off');
    else
        % If no orbit control, display message in second row plots
        for i = 4:6
            subplot(nb_row, nb_col, i);
            text(0.5, 0.5, 'No orbit control data available', 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
            title(['Plot ' num2str(i)]);
        end
    end
    
    %% Row 3: Fuel Plots
    if has_fuel_tanks
        time_vec = mission.true_time.store.time(1:kd);
        
        % Plot 7: Remaining fuel capacity
        subplot(nb_row, nb_col, 7);
        hold on;
        
        % Plot each fuel tank separately
        total_initial_capacity = 0;
        
        for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
            % Get fuel tank data
            tank = mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank};
            fuel_mass = tank.store.instantaneous_fuel_mass(1:kd);
            initial_capacity = tank.maximum_capacity;
            total_initial_capacity = total_initial_capacity + initial_capacity;
            
            % Plot individual tank level - simple line plot
            plot(time_vec, fuel_mass, '-', 'LineWidth', 2, 'Color', colors{i_tank}, 'DisplayName', ['Tank ' num2str(i_tank)]);
            
            % Plot maximum capacity line
            plot([time_vec(1), time_vec(end)], [initial_capacity, initial_capacity], ':', 'Color', colors{i_tank}, 'LineWidth', 1, 'DisplayName', ['Max ' num2str(i_tank)]);
        end
        
        % Calculate and plot total fuel remaining for multiple tanks
        if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 1
            total_fuel = zeros(kd, 1);
            for i_tank = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank
                total_fuel = total_fuel + mission.true_SC{i_SC}.true_SC_fuel_tank{i_tank}.store.instantaneous_fuel_mass(1:kd);
            end
            plot(time_vec, total_fuel, '-k', 'LineWidth', 2.5, 'DisplayName', 'Total');
            
            % Plot total capacity line
            plot([time_vec(1), time_vec(end)], [total_initial_capacity, total_initial_capacity], ':k', 'LineWidth', 1.5, 'DisplayName', 'Total Cap');
        end
        
        % Add percentage labels on the right Y-axis
        if total_initial_capacity > 0
            % Restore yyright axis for percentage display
            ax1 = gca;
            yyaxis right;
            
            if mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_fuel_tank > 1
                % For multiple tanks, show total percentage
                percentage = (total_fuel / total_initial_capacity) * 100;
                plot(time_vec, percentage, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1, 'DisplayName', '% Remaining');
                ylabel('Remaining Fuel [%]');
            else
                % For single tank
                percentage = (fuel_mass / initial_capacity) * 100;
                plot(time_vec, percentage, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1, 'DisplayName', '% Remaining');
                ylabel('Remaining Fuel [%]');
            end
            
            yyaxis left; % Switch back to left axis for remaining operations
        end
        
        % Add legend and labels
        legend('Location', 'northeast');
        grid on; xlabel('Time [sec]'); ylabel('Fuel Mass [kg]');
        title('Propellant Remaining');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
        
        % Plot 8: Chemical Thruster Consumption
        subplot(nb_row, nb_col, 8);
        hold on;
        
        % Calculate consumption by chemical thrusters
        chemical_consumption = zeros(kd, 1);
        if has_chemical_thrusters
            for i_CT = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_chemical_thruster
                thruster = mission.true_SC{i_SC}.true_SC_chemical_thruster{i_CT};
                if isfield(thruster.store, 'total_fuel_consumed')
                    chemical_consumption = chemical_consumption + thruster.store.total_fuel_consumed(1:kd);
                end
            end
            
            if any(chemical_consumption > 0)
                plot(time_vec, chemical_consumption, '-r', 'LineWidth', 2, 'DisplayName', 'Chemical');
                
                % Add percentage axis on the right
                if total_initial_capacity > 0
                    yyaxis right;
                    consumption_percentage = (chemical_consumption / total_initial_capacity) * 100;
                    plot(time_vec, consumption_percentage, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1, 'DisplayName', '% Used');
                    ylabel('% of Total Capacity');
                    yyaxis left;
                end
            else
                text(0.5, 0.5, 'No chemical thruster consumption yet', 'HorizontalAlignment', 'center', 'FontSize', 12);
            end
        else
            text(0.5, 0.5, 'No chemical thrusters present', 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
        
        % Add legend and labels
        legend('Location', 'northwest');
        grid on; xlabel('Time [sec]'); ylabel('Fuel Consumed [kg]');
        title('Chemical Thruster Propellant Consumption');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
        
        % Plot 9: Micro Thruster Consumption
        subplot(nb_row, nb_col, 9);
        hold on;
        
        % Calculate total consumption by micro thrusters
        micro_consumption = zeros(kd, 1);
        if has_micro_thrusters
            % We need to handle the different time storage indexing for attitude thrusters
            kd_attitude = min(mission.storage.k_storage_attitude, size(mission.true_time.store.time_attitude, 1));
            if kd_attitude > 0
                % Get time vector for attitude data
                time_vec_attitude = mission.true_time.store.time_attitude(1:kd_attitude);
                
                % First collect micro thruster consumption at attitude storage rate
                micro_consumption_raw = zeros(kd_attitude, 1);
                
                for i_MT = 1:mission.true_SC{i_SC}.true_SC_body.num_hardware_exists.num_micro_thruster
                    thruster = mission.true_SC{i_SC}.true_SC_micro_thruster{i_MT};
                    if isfield(thruster.store, 'total_fuel_consumed')
                        micro_consumption_raw = micro_consumption_raw + thruster.store.total_fuel_consumed(1:kd_attitude);
                    end
                end
                
                % Then interpolate to orbit control storage rate if needed
                if length(time_vec) ~= length(time_vec_attitude)
                    micro_consumption = interp1(time_vec_attitude, micro_consumption_raw, time_vec, 'linear', 'extrap');
                else
                    micro_consumption = micro_consumption_raw;
                end
                
                % Plot consumption if thrusters exist and have consumed fuel
                if any(micro_consumption > 0)
                    plot(time_vec, micro_consumption, '-b', 'LineWidth', 2, 'DisplayName', 'Micro');
                    
                    % Add percentage axis on the right
                    if total_initial_capacity > 0
                        yyaxis right;
                        consumption_percentage = (micro_consumption / total_initial_capacity) * 100;
                        plot(time_vec, consumption_percentage, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1, 'DisplayName', '% Used');
                        ylabel('% of Total Capacity');
                        yyaxis left;
                    end
                else
                    text(0.5, 0.5, 'No micro thruster consumption yet', 'HorizontalAlignment', 'center', 'FontSize', 12);
                end
            else
                text(0.5, 0.5, 'No attitude data available', 'HorizontalAlignment', 'center', 'FontSize', 12);
            end
        else
            text(0.5, 0.5, 'No micro thrusters present', 'HorizontalAlignment', 'center', 'FontSize', 12);
        end
        
        % Add legend and labels
        legend('Location', 'northwest');
        grid on; xlabel('Time [sec]'); ylabel('Fuel Consumed [kg]');
        title('Micro Thruster Propellant Consumption');
        set(gca, 'FontSize', mission.storage.plot_parameters.standard_font_size, 'FontName', mission.storage.plot_parameters.standard_font_type);
        hold off;
    else
        % If no fuel tanks, display message in third row plots
        for i = 7:9
            subplot(nb_row, nb_col, i);
            text(0.5, 0.5, 'No fuel tank data available', 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
            title(['Plot ' num2str(i)]);
        end
    end
    
    %% Super Title
    sgtitle(['SC ', num2str(i_SC), ' Orbital Control Performance'], ...
        'fontsize', mission.storage.plot_parameters.title_font_size, ...
        'FontName', mission.storage.plot_parameters.standard_font_type);
    
    %% Save Plot
    if mission.storage.plot_parameters.flag_save_plots == 1
        saveas(plot_handle, [mission.storage.output_folder, mission.name, '_SC', num2str(i_SC), '_Orbit_Control.png']);
    end
end