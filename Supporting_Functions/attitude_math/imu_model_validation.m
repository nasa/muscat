%% MuSCAT IMU Validation
% Validation of "Rate bias" model

clear;

% Add required paths
currpath = fileparts(mfilename("fullpath"));
addpath(currpath);
addpath(genpath(fullfile(currpath,'../.')))
addpath(genpath(fullfile(currpath,'../../MuSCAT_Supporting_Files')))

%% Constants
rps2dps = 180/pi;
rps2dph = rps2dps*3600;

%% Validation Test Inputs
uts_to_run = 6;
rng(0);
i_SC = 1;
t_final = 100; % [sec] Final time for runs
time_step_attitude = 0.1; % [sec] Attitude time step
utIdx = 0;

%% IMU Configuration
init_data_imu_base = [];
init_data_imu_base.instantaneous_power_consumed = 0.6; % [W] https://www.micro-a.net/imu-tmpl.html
init_data_imu_base.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb] : 3 angular velocity + 1 time vector, each of 16-bit depth
init_data_imu_base.mode_true_SC_imu_selector = 'Rate bias';
init_data_imu_base.measurement_noise = 9.7e-5; % [rad/sec]
init_data_imu_base.location = [0 0 0]; % [m]
init_data_imu_base.orientation = [1 0 0]; % [unit vector]
init_data_imu_base.measurement_wait_time = 0.1;

%% Unit Test #1 - Model Defaults
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude);

    nImu = 100;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.measurement_wait_time = min([0.1, mission{utIdx}.true_time.time_step_attitude]); % [sec]

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Unit Test #2 - Change in time step size
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    time_step_attitude_ut2 = 0.5;
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude_ut2);

    nImu = 100;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.measurement_wait_time = min([0.1, mission{utIdx}.true_time.time_step_attitude]); % [sec]
    init_data_imu.bias_long_term_stab = 0; % No long term bias stability

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Unit Test #3 - Change in time step size below measurement_wait_time
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    time_step_attitude_ut2 = 0.02;
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude_ut2);

    nImu = 100;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.bias_long_term_stab = 0; % No long term bias stability

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Unit Test #4 - Non-default ARW, RRW
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude);

    nImu = 100;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.measurement_wait_time = min([0.1, mission{utIdx}.true_time.time_step_attitude]); % [sec]
    init_data_imu.bias_long_term_stab = 0; % No long term bias stability
    init_data_imu.arw_psd = (0.003*pi/180/sqrt(3600))^2;     % [rad^2/s] Angle Random Walk (rate white noise) PSD height
    init_data_imu.rrw_psd = (2*pi/180/3600)^2/(24*3600); % [(rad/s)^2/s] Rate (bias) Random Walk PSD height

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Unit Test #5 - Non-zero initial bias
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude);

    nImu = 1;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.measurement_wait_time = min([0.1, mission{utIdx}.true_time.time_step_attitude]); % [sec]
    init_data_imu.bias_long_term_stab = 0; % No long term bias stability

    % Set non-zero initial bias
    ut_init_bias = [1 2 3]*pi/180/3600;
    init_data_imu.initial_bias = ut_init_bias; % [rad/s] Initial rate bias

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Unit Test #6 - Long-term bias stability
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    t_final_ut6 = 100;
    mission{utIdx} = [];
    mission{utIdx} = initModel(i_SC, t_final_ut6, time_step_attitude);

    nImu = 100;
    init_data_imu = init_data_imu_base;
    init_data_imu.rng_seed = utIdx;
    init_data_imu.measurement_wait_time = min([0.1, mission{utIdx}.true_time.time_step_attitude]); % [sec]

    % Set non-zero initial bias
    ut_init_bias = [1 2 3]*pi/180/3600;
    init_data_imu.initial_bias = ut_init_bias; % [rad/s] Initial rate bias
    init_data_imu.rrw_psd = (100*pi/180/3600)^2/(100); % [(rad/s)^2/s] Rate (bias) Random Walk PSD height
    init_data_imu.bias_long_term_stab = 25*pi/180/3600; % No long term bias stability

    for i_HW = 1:nImu
        mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW} = True_SC_IMU(init_data_imu, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running IMU UT#%d\n', utIdx)
    mission{utIdx} = runImuModel(mission{utIdx}, i_SC, nImu);
    fprintf('\n');
end

%% Plot UT#1-4 results
plotUts = 1:4;
validUtIdxs = ismember(plotUts,uts_to_run);
if any(validUtIdxs)
    for utIdx = plotUts(validUtIdxs)

        nImu = length(mission{utIdx}.true_SC{i_SC}.true_SC_imu);
        dtStr = sprintf('%st = %g sec', '\Delta', mission{utIdx}.true_time.time_step_attitude);
        t = mission{utIdx}.true_time.store.time_attitude;

        % Compute bounds from RRW PSD
        arw_sigma = mission{utIdx}.true_SC{i_SC}.true_SC_imu{1}.arw_sigma; % [rad/s]
        arw_psd = mission{utIdx}.true_SC{i_SC}.true_SC_imu{1}.arw_psd; % [rad^2/s]
        rrw_psd = mission{utIdx}.true_SC{i_SC}.true_SC_imu{1}.rrw_psd; % [(rad/s)^2/s]

        rate_noise_bound = 3*arw_sigma*ones(size(t)); % [rad/s]
        bias_walk_bound = 3*sqrt(rrw_psd*t); % [rad/s]

        % Plot the rate minus the bias (rate with white noise
        figure('Name',sprintf('UT#%d Rate Noise',utIdx));
        tiledlayout(3,1);
        axIdx = 0;
        ax = [];

        for ii = 1:3
            axIdx = axIdx + 1;
            ax(axIdx) = nexttile;

            for i_HW = 1:nImu
                rate = mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW}.store.measurement_vector(:,ii);
                bias = mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW}.store.rate_bias(:,ii);
                plot(t,(rate - bias)*rps2dph);
                hold on;
            end
            hb = plot(t,rate_noise_bound*rps2dph,'k--');
            plot(t,-rate_noise_bound*rps2dph,'k--');
            grid on;
            legend(hb,'3-\sigma ARW Bound','Location','northwest');
            titleStr = sprintf('IMU Gyro %d Unbiased Rate for %d IMUs',ii, nImu);
            if ii == 1
                titleStr = sprintf('%s\n%s',dtStr,titleStr);
            end
            title(titleStr);
            xlabel('Time (sec)');
            ylabel('Rate (deg/hr)');
        end

        % Plot the rate bias
        figure('Name',sprintf('UT#%d Rate Bias',utIdx));
        tiledlayout(3,1);
        axIdx = 0;
        ax = [];

        for ii = 1:3
            axIdx = axIdx + 1;
            ax(axIdx) = nexttile;

            for i_HW = 1:nImu
                bias = mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW}.store.rate_bias(:,ii);
                plot(t,bias*rps2dph);
                hold on;
            end
            hb = plot(t,bias_walk_bound*rps2dph,'k--');
            plot(t,-bias_walk_bound*rps2dph,'k--');
            grid on;
            legend(hb,'3-\sigma RRW Bound','Location','northwest');
            titleStr = sprintf('IMU Gyro %d Bias for %d IMUs',ii, nImu);
            if ii == 1
                titleStr = sprintf('%s\n%s',dtStr,titleStr);
            end
            title(titleStr);
            xlabel('Time (sec)');
            ylabel('Bias (deg/hr)');
        end

    end
end

%% Plot UT#5-6 results
plotUts = 5:6;
validUtIdxs = ismember(plotUts,uts_to_run);
if any(validUtIdxs)
    for utIdx = plotUts(validUtIdxs)

        nImu = length(mission{utIdx}.true_SC{i_SC}.true_SC_imu);
        dtStr = sprintf('%st = %g sec', '\Delta', mission{utIdx}.true_time.time_step_attitude);
        t = mission{utIdx}.true_time.store.time_attitude;

        % Compute bounds from RRW PSD
        rrw_psd = mission{utIdx}.true_SC{i_SC}.true_SC_imu{1}.rrw_psd; % [(rad/s)^2/s]

        init_bias = ones(length(t),1)*ut_init_bias(:).';
        bias_long_term_stab = mission{utIdx}.true_SC{i_SC}.true_SC_imu{1}.bias_long_term_stab*ones(size(t));
        bias_walk_bound = 3*sqrt(rrw_psd*t); % [rad/s]

        % Plot the rate bias
        figure('Name',sprintf('UT#%d Rate Bias',utIdx));
        tiledlayout(3,1);
        axIdx = 0;
        ax = [];

        for ii = 1:3
            axIdx = axIdx + 1;
            ax(axIdx) = nexttile;

            for i_HW = 1:nImu
                bias = mission{utIdx}.true_SC{i_SC}.true_SC_imu{i_HW}.store.rate_bias(:,ii);
                plot(t,bias*rps2dph);
                hold on;
            end
            hb = plot(t,(init_bias(:,ii)+bias_walk_bound)*rps2dph,'k--');
            plot(t,(init_bias(:,ii)-bias_walk_bound)*rps2dph,'k--');
            if utIdx == 5
                hi = plot(t,init_bias(:,ii)*rps2dph,'r-.');
                legStr = 'Input Initial Bias';
            elseif utIdx == 6
                hi = plot(t,(init_bias(:,ii) + 3*bias_long_term_stab)*rps2dph,'m-.');
                plot(t,(init_bias(:,ii) - 3*bias_long_term_stab)*rps2dph,'m-.');
                legStr = '3\sigma Long-term Stability';
            end
            grid on;
            legend([hb, hi],'3-\sigma RRW Bound',legStr,'Location','northwest');
            titleStr = sprintf('IMU Gyro %d Bias for %d IMUs',ii, nImu);
            if ii == 1
                titleStr = sprintf('%s\n%s',dtStr,titleStr);
            end
            title(titleStr);
            xlabel('Time (sec)');
            ylabel('Bias (deg/hr)');
        end

    end
end

%% Function to initialize a spacecraft
function mission = initModel(i_SC, t_final, time_step_attitude)
%% Mission Definition
mission = [];
mission.name = 'IBEAM';        % Name of the Mission
mission.num_SC = 1;            % Number of Spacecraft
mission.num_target = 1;        % Number of Target bodies
mission.frame = 'Absolute';    % Frame type: 'Absolute', 'Relative', or 'Combined'
mission.flag_stop_sim = 0;     % Boolean flag to stop simulation if needed

%% Time Configuration
init_data = [];
init_data.t_initial = 0;                                    % [sec] Initial time
init_data.t_final = t_final;                                % [sec] Final time 
init_data.time_step = 5;                                    % [sec] Simulation time step
init_data.t_initial_date_string = '01-APR-2027 00:00:00';   % Format = [DD-MMM-YYYY HH:MM:SS]
init_data.time_step_attitude = time_step_attitude;          % [sec] Time step for attitude dynamics
init_data.time_step_position_array = 1;                     % [sec] Time step for position array 
mission.true_time = True_Time(init_data);

%% Storage Configuration
init_data = [];
init_data.time_step_storage = 0;
init_data.time_step_storage_attitude = 0;
init_data.flag_realtime_plotting = 0;      % [Boolean] Show mission data and attitude during sim
init_data.wait_time_realtime_plotting = 1*60; % [sec] Wait time between plots (Optional)
init_data.flag_save_plots = 1;             % [Boolean] 1: Save them (takes little time), 0: Doesnt save them
init_data.flag_save_video = 0;             % [Boolean] 1: Save them (takes more time), 0: Doesnt save them
init_data.quiver_auto_scale_factor = 1; % (Optional)
mission.storage = Storage(init_data, mission);

% Set font size for plots
mission.storage.plot_parameters.standard_font_size = 15;

% Initialize time storage
func_initialize_time_store(mission.true_time, mission);

%% Spacecraft Initialization
mission.true_SC{i_SC} = [];

%% Initialize First Spacecraft's Rate
mission.true_SC{i_SC}.true_SC_adc.angular_velocity = [0 0 0];

%% Initialize First Spacecraft's Power
init_data = [];
init_data.power_loss_rate = 0.05; % [float] 5% power loss in distribution and conversion
mission.true_SC{i_SC}.true_SC_power = True_SC_Power(init_data, mission);

%% Initialize First Spacecraft's Data
init_data = [];
init_data.mode_true_SC_data_handling_selector = 'Generic';
mission.true_SC{i_SC}.true_SC_data_handling = True_SC_Data_Handling(init_data, mission);
end

%% Function for running the time loop
function mission_out = runImuModel(mission, i_SC, nImu)
% Execute the model
for k = 1:1:mission.true_time.num_time_steps

    % Update Time, Date
    func_update_true_time_date(mission.true_time, k);

    % Update Storage
    func_update_storage_flag(mission.storage, mission);
    func_update_time_store(mission.true_time, mission);

    % Attitude Dynamics Loop (ADL)
    for k_attitude = 1:1:mission.true_time.num_time_steps_attitude

        % Update Time in ADL
        func_update_true_time_attitude(mission.true_time, k_attitude);

        % Update Storage in ADL
        func_update_storage_flag_attitude(mission.storage, mission);
        func_update_time_store_attitude(mission.true_time, mission);

        % Update SC IMU in ADL
        for i_HW = 1:nImu
            func_main_true_SC_imu(mission.true_SC{i_SC}.true_SC_imu{i_HW}, mission, i_SC);

            % Re-init IMU flag executive
            mission.true_SC{i_SC}.true_SC_imu{i_HW}.flag_executive = 1;
        end

    end

end
mission_out = mission;
end