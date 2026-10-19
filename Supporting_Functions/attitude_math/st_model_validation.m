%% MuSCAT Star Tracker Validation
% Validation of "Bias + First Order Gauss Markov" model
%
% Error model (sensor frame, boresight = +Z):
%   total_angle_SF = NEA + constant_bias + FOGM_drift
% where
%   NEA           : white noise,  1-sigma = angle_sigma_SF        [cross;cross;boresight]
%   constant_bias : sampled once, 1-sigma = bias_sigma_SF
%   FOGM_drift    : first-order Gauss-Markov, correlation time fogm_tau,
%                   steady-state 1-sigma = fogm_sigma_SF, initial value
%                   sampled from that steady-state distribution
% The measured quaternion is  q_meas = qmult( vec2q(total_angle_BF), q_true ).

clear;

% Add required paths
currpath = fileparts(mfilename("fullpath"));
addpath(currpath);
addpath(genpath(fullfile(currpath,'../.')))
addpath(genpath(fullfile(currpath,'../../MuSCAT_Supporting_Files')))

%% Constants
rad2arcsec = 180/pi*3600;

%% Validation Test Inputs
uts_to_run = 1:6;
rng(0);
i_SC = 1;
t_final = 100;             % [sec] Final time for runs
time_step_attitude = 0.1;  % [sec] Attitude time step
trueAtt = [0 0 0 1];       % True (constant) attitude quaternion, scalar-last
utIdx = 0;

%% Star Tracker Configuration (base)
init_data_st_base = [];
init_data_st_base.instantaneous_power_consumed = 1.5; % [W]
init_data_st_base.instantaneous_data_generated_per_sample = (4 + 1)*16e-3; % [kb]
init_data_st_base.mode_true_SC_star_tracker_selector = 'Bias + First Order Gauss Markov';
init_data_st_base.measurement_wait_time = 0.1; % [sec]
init_data_st_base.field_of_view = 35; % [deg] (unused in this mode)
init_data_st_base.location = [0 0 0]; % [m]
init_data_st_base.alignment_BF_SF = eye(3);
init_data_st_base.boresight_SF = [0; 0; 1];
% Noise-equivalent angle (constructor zeroes these if not provided)
init_data_st_base.measurement_noise_cross_boresight = 4.2/3600*pi/180; % [rad]
init_data_st_base.measurement_noise_about_boresight = 34/3600*pi/180;  % [rad]
% Bias and FOGM left at class defaults (bias 25/60 arcsec, FOGM 3/15 arcsec, tau 1800 s)
%
% Note on seeding: True_SC_Star_Tracker's rand_stream property default is a
% single RandStream handle shared by all instances, and the constructor resets
% it to init_data.rng_seed. Reusing one seed for every unit therefore makes all
% units sample the *identical* constant bias/initial FOGM. Each unit is given a
% distinct rng_seed below so the ensemble has genuine spread.

%% Unit Test #1 - Model Defaults
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude, trueAtt);

    nSt = 100;
    init_data_st = init_data_st_base;

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (defaults)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);
    fprintf('\n');
end

%% Unit Test #2 - Larger attitude time step
% FOGM steady-state std must be invariant to the time step.
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = initModel(i_SC, t_final, 0.5, trueAtt);

    nSt = 100;
    init_data_st = init_data_st_base;
    init_data_st.measurement_wait_time = 0.5;

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (dt = 0.5 s)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);
    fprintf('\n');
end

%% Unit Test #3 - Attitude time step below measurement_wait_time
% sim_dt = max(time_step_attitude, measurement_wait_time), so the FOGM step
% is governed by measurement_wait_time here.
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = initModel(i_SC, t_final, 0.02, trueAtt);

    nSt = 100;
    init_data_st = init_data_st_base;
    init_data_st.measurement_wait_time = 0.1; % > time_step_attitude

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (dt = 0.02 s < wait = 0.1 s)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);
    fprintf('\n');
end

%% Unit Test #4 - Non-default NEA / bias / FOGM parameters
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude, trueAtt);

    nSt = 100;
    init_data_st = init_data_st_base;
    init_data_st.measurement_noise_cross_boresight = 2/3600*pi/180;  % [rad]
    init_data_st.measurement_noise_about_boresight = 10/3600*pi/180; % [rad]
    init_data_st.bias_sigma_cross     = 10/3600*pi/180; % [rad]
    init_data_st.bias_sigma_boresight = 40/3600*pi/180; % [rad]
    init_data_st.fogm_sigma_cross     = 5/3600*pi/180;  % [rad]
    init_data_st.fogm_sigma_boresight = 20/3600*pi/180; % [rad]
    init_data_st.fogm_tau = 900; % [sec]

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (non-default parameters)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);
    fprintf('\n');
end

%% Unit Test #5 - Specified constant bias + non-identity alignment (bias-path check)
% Zero NEA and zero FOGM so the only error is the specified constant bias.
% The extracted sensor-frame error must equal the input bias exactly, which
% also validates the SF->BF->quaternion->extraction round trip through a
% non-identity alignment.
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    mission{utIdx} = initModel(i_SC, t_final, time_step_attitude, trueAtt);

    nSt = 1;
    init_data_st = init_data_st_base;
    init_data_st.measurement_noise_cross_boresight = 0; % zero NEA
    init_data_st.measurement_noise_about_boresight = 0;
    init_data_st.fogm_sigma_cross     = 0; % zero FOGM
    init_data_st.fogm_sigma_boresight = 0;
    ut5_bias_SF = [30 -20 50]/3600*pi/180; % [rad] specified constant bias in sensor frame
    init_data_st.bias_SF = ut5_bias_SF;
    % Non-identity alignment (45 deg about SF x-axis)
    ca = cosd(45); sa = sind(45);
    init_data_st.alignment_BF_SF = [1 0 0; 0 ca -sa; 0 sa ca];

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (specified bias, non-identity alignment)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);

    % Verify extracted error equals the input bias
    st = mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{1};
    errSF = extractSensorFrameError(st.store.measurement_vector, trueAtt, st.alignment_BF_SF);
    errSF = errSF(all(~isnan(errSF),2),:); % drop non-measurement samples
    meanErr = mean(errSF,1); % [arcsec]
    maxDev  = max(abs(errSF - (ut5_bias_SF*rad2arcsec)),[],1); % [arcsec]
    fprintf('  Input bias  [arcsec]: [% .4f % .4f % .4f]\n', ut5_bias_SF*rad2arcsec);
    fprintf('  Mean error  [arcsec]: [% .4f % .4f % .4f]\n', meanErr);
    fprintf('  Max |error - bias|  : [% .3e % .3e % .3e] arcsec\n', maxDev);
    if all(maxDev < 1e-6)
        fprintf('  PASS: constant bias applied exactly through alignment.\n');
    else
        fprintf('  CHECK: residual exceeds tolerance.\n');
    end
    fprintf('\n');
end

%% Unit Test #6 - FOGM growth from zero, correlation time, and bounding
% Isolate the FOGM (zero NEA, zero bias) and start it at zero so the drift is
% seen growing from 0 and then bounded by the steady-state envelope. Verify:
%   1. growth follows 3*sigma_ss*sqrt(1-exp(-2t/tau)) toward the 3-sigma bound
%   2. steady-state std (stationary tail) matches fogm_sigma_SF
%   3. normalized autocorrelation (stationary tail) decays as exp(-lag/fogm_tau)
utIdx = utIdx + 1;
if ismember(utIdx,uts_to_run)
    dt6 = 1;              % [sec] attitude time step
    tau6 = 1800;         % [sec] FOGM correlation time
    t_final6 = 6*tau6;   % [sec] several correlation times
    mission{utIdx} = initModel(i_SC, t_final6, dt6, trueAtt);

    nSt = 50;
    init_data_st = init_data_st_base;
    init_data_st.measurement_wait_time = dt6;
    init_data_st.measurement_noise_cross_boresight = 0; % isolate FOGM
    init_data_st.measurement_noise_about_boresight = 0;
    init_data_st.bias_sigma_cross     = 0;
    init_data_st.bias_sigma_boresight = 0;
    init_data_st.fogm_tau = tau6;
    init_data_st.initial_fogm_SF = [0 0 0]; % start FOGM at zero to show growth

    for i_HW = 1:nSt
        init_data_st.rng_seed = 1000*utIdx + i_HW; % distinct seed per unit (see note below)
        mission{utIdx}.true_SC{i_SC}.true_SC_star_tracker{i_HW} = ...
            True_SC_Star_Tracker(init_data_st, mission{utIdx}, i_SC, i_HW);
    end

    fprintf('Running Star Tracker UT#%d (FOGM correlation time)\n', utIdx)
    mission{utIdx} = runStarTrackerModel(mission{utIdx}, i_SC, nSt);
    fprintf('\n');
end

%% Plot UT#1-4 results (NEA + FOGM ensembles with 3-sigma bounds)
plotUts = 1:4;
validUtIdxs = ismember(plotUts,uts_to_run);
for utIdx = plotUts(validUtIdxs)

    [t, totErr, fogm, biasAll, st1] = collectStData(mission{utIdx}, i_SC, trueAtt, rad2arcsec);
    nSt = size(totErr,3);
    dtStr = sprintf('%st = %g sec', '\Delta', mission{utIdx}.true_time.time_step_attitude);

    % Per-axis 3-sigma bounds (arcsec)
    nea_bound  = 3*st1.angle_sigma_SF(:).'*rad2arcsec;
    fogm_bound = 3*st1.fogm_sigma_SF(:).'*rad2arcsec;
    bias_bound = 3*[st1.bias_sigma_cross st1.bias_sigma_cross st1.bias_sigma_boresight]*rad2arcsec;

    % Noise-equivalent angle = total error minus bias minus FOGM
    neaErr = totErr - reshape(biasAll.',1,3,nSt) - fogm;

    plot3PanelEnsemble(t, neaErr, nea_bound, sprintf('UT#%d NEA',utIdx), ...
        sprintf('Attitude Error (NEA only)'), dtStr, '3-\sigma NEA Bound');
    plot3PanelEnsemble(t, fogm, fogm_bound, sprintf('UT#%d FOGM',utIdx), ...
        sprintf('FOGM Drift'), dtStr, '3-\sigma FOGM SS Bound');
    if nSt >= 10
        plotBiasHistogram(biasAll, bias_bound, sprintf('UT#%d Bias',utIdx), dtStr);
    end

    % Print empirical vs expected 1-sigma
    printStats(utIdx, neaErr, fogm, biasAll, ...
        st1.angle_sigma_SF(:).'*rad2arcsec, st1.fogm_sigma_SF(:).'*rad2arcsec, bias_bound/3);
end

%% Plot UT#6 result (FOGM growth from zero + autocorrelation)
if ismember(6,uts_to_run)
    [t, ~, fogm, ~, st1] = collectStData(mission{6}, i_SC, trueAtt, rad2arcsec);
    nSt = size(fogm,3);
    dt = st1.sim_dt;
    tau = st1.fogm_tau;
    fogm_ss_exp = st1.fogm_sigma_SF(:).'*rad2arcsec;   % [arcsec] steady-state 1-sigma
    dtStr = sprintf('%st = %g sec', '\Delta', dt);

    % FOGM starts at zero: plot the growth with the theoretical envelope and
    % the asymptotic 3-sigma steady-state bound
    plotFogmGrowth(t, fogm, fogm_ss_exp, tau, sprintf('UT#%d FOGM Growth',6), dtStr);

    % Steady-state statistics use the stationary tail (after the growth transient)
    tailStart = 2*tau;                 % [sec] ~99% of steady-state variance reached
    tailIdx = t >= tailStart;
    fogm_tail = fogm(tailIdx,:,:);

    % Steady-state std check on the tail (permute so axis is last before flattening)
    fogm_ss_emp = std(reshape(permute(fogm_tail,[1 3 2]),[],3),0,1,'omitnan'); % [arcsec]
    fprintf('UT#6 FOGM steady-state 1-sigma [arcsec] (tail t >= %g s):\n', tailStart);
    fprintf('  expected  : [% .3f % .3f % .3f]\n', fogm_ss_exp);
    fprintf('  empirical : [% .3f % .3f % .3f]\n', fogm_ss_emp);

    % Ensemble-averaged normalized autocorrelation on the stationary tail
    [lagTime, acfMean, acfTheory] = fogmAutocorr(fogm_tail/rad2arcsec, dt, tau, 3*tau);

    figure('Name','UT#6 FOGM Autocorrelation');
    plot(lagTime, acfMean, 'b', 'LineWidth', 1.5); hold on; grid on;
    plot(lagTime, acfTheory, 'k--', 'LineWidth', 1.5);
    plot([tau tau], [0 1], 'r-.');
    plot([lagTime(1) lagTime(end)], exp(-1)*[1 1], 'r:');
    legend('Empirical (ensemble mean)', 'exp(-lag/\tau)', ...
        sprintf('\\tau = %g s', tau), '1/e', 'Location','northeast');
    title(sprintf('FOGM Normalized Autocorrelation (%d units, \\Deltat = %g s)', nSt, dt));
    xlabel('Lag (sec)'); ylabel('Normalized autocorrelation');
    fprintf('\n');
end

%% ------------------------------------------------------------------------
%% Local Functions
%% ------------------------------------------------------------------------

function mission = initModel(i_SC, t_final, time_step_attitude, trueAtt)
%% Mission Definition
mission = [];
mission.name = 'IBEAM';
mission.num_SC = 1;
mission.num_target = 1;
mission.frame = 'Absolute';
mission.flag_stop_sim = 0;

%% Time Configuration
init_data = [];
init_data.t_initial = 0;
init_data.t_final = t_final;
init_data.time_step = 5;
init_data.t_initial_date_string = '01-APR-2027 00:00:00';
init_data.time_step_attitude = time_step_attitude;
init_data.time_step_position_array = 1;
mission.true_time = True_Time(init_data);

%% Storage Configuration
init_data = [];
init_data.time_step_storage = 0;
init_data.time_step_storage_attitude = 0;
init_data.flag_realtime_plotting = 0;
init_data.wait_time_realtime_plotting = 1*60;
init_data.flag_save_plots = 1;
init_data.flag_save_video = 0;
init_data.quiver_auto_scale_factor = 1;
mission.storage = Storage(init_data, mission);

mission.storage.plot_parameters.standard_font_size = 15;

func_initialize_time_store(mission.true_time, mission);

%% Spacecraft Initialization
mission.true_SC{i_SC} = [];

%% Attitude (constant true attitude read by the star tracker)
mission.true_SC{i_SC}.true_SC_adc.attitude = trueAtt;
mission.true_SC{i_SC}.true_SC_adc.angular_velocity = [0 0 0];

%% Power
init_data = [];
init_data.power_loss_rate = 0.05;
mission.true_SC{i_SC}.true_SC_power = True_SC_Power(init_data, mission);

%% Data Handling
init_data = [];
init_data.mode_true_SC_data_handling_selector = 'Generic';
mission.true_SC{i_SC}.true_SC_data_handling = True_SC_Data_Handling(init_data, mission);
end

function mission_out = runStarTrackerModel(mission, i_SC, nSt)
% Execute the model over the simulation time loop
for k = 1:1:mission.true_time.num_time_steps

    func_update_true_time_date(mission.true_time, k);

    func_update_storage_flag(mission.storage, mission);
    func_update_time_store(mission.true_time, mission);

    % Attitude Dynamics Loop (ADL)
    for k_attitude = 1:1:mission.true_time.num_time_steps_attitude

        func_update_true_time_attitude(mission.true_time, k_attitude);

        func_update_storage_flag_attitude(mission.storage, mission);
        func_update_time_store_attitude(mission.true_time, mission);

        for i_HW = 1:nSt
            func_main_true_SC_star_tracker(mission.true_SC{i_SC}.true_SC_star_tracker{i_HW}, mission, i_SC);
            % Re-arm the sensor for the next step
            mission.true_SC{i_SC}.true_SC_star_tracker{i_HW}.flag_executive = 1;
        end

    end

end
mission_out = mission;
end

function [t, totErr, fogm, biasAll, st1] = collectStData(mission, i_SC, trueAtt, rad2arcsec)
% Gather per-unit stored data and extract sensor-frame errors (arcsec)
t = mission.true_time.store.time_attitude;
nSt = length(mission.true_SC{i_SC}.true_SC_star_tracker);
st1 = mission.true_SC{i_SC}.true_SC_star_tracker{1};

nSamp = length(t);
totErr  = nan(nSamp, 3, nSt);
fogm    = nan(nSamp, 3, nSt);
biasAll = nan(nSt, 3);

for i_HW = 1:nSt
    st = mission.true_SC{i_SC}.true_SC_star_tracker{i_HW};
    e = extractSensorFrameError(st.store.measurement_vector, trueAtt, st.alignment_BF_SF);
    totErr(:,:,i_HW) = e(1:nSamp,:);
    fogm(:,:,i_HW)   = st.store.fogm_SF(1:nSamp,:) * rad2arcsec;
    biasAll(i_HW,:)  = st.bias_SF * rad2arcsec;
end
end

function errSF_arcsec = extractSensorFrameError(measStore, trueAtt, alignment_BF_SF)
% Recover the sensor-frame small-angle error (arcsec) from stored quaternions.
% q_meas = qmult(error_quat_B, q_true)  ->  error_quat_B = qmult(q_meas, qinv(q_true))
% Rows that are not unit quaternions (e.g. the initial zero placeholder) -> NaN.
rad2arcsec = 180/pi*3600;
nSamp = size(measStore,1);
errSF_arcsec = nan(nSamp,3);

qmag = sqrt(sum(measStore.^2,2));
valid = abs(qmag - 1) < 1e-6;
if ~any(valid)
    return;
end

mv = measStore(valid,:);
errQ = qmult(mv, qinv(trueAtt));      % Nx4 (implicit expansion on trueAtt)
[v, th] = q2vec(errQ);                % v: Nx3 unit axis, th: Nx1 angle
rotvec_BF = v .* th;                  % Nx3 rotation vector in body frame
rotvec_SF = (alignment_BF_SF.' * rotvec_BF.').'; % body -> sensor frame
errSF_arcsec(valid,:) = rotvec_SF * rad2arcsec;
end

function plot3PanelEnsemble(t, data3d, boundVec, figName, titleBase, dtStr, boundLabel)
axisNames = {'X (cross)','Y (cross)','Z (boresight)'};
nUnits = size(data3d,3);
figure('Name',figName);
tiledlayout(3,1);
for ii = 1:3
    nexttile;
    plot(t, squeeze(data3d(:,ii,:)));
    hold on; grid on;
    hb = plot(t, boundVec(ii)*ones(size(t)), 'k--', 'LineWidth', 1.2);
    plot(t, -boundVec(ii)*ones(size(t)), 'k--', 'LineWidth', 1.2);
    legend(hb, boundLabel, 'Location','northeast');
    ttl = sprintf('%s - %s (%d units)', titleBase, axisNames{ii}, nUnits);
    if ii == 1
        ttl = sprintf('%s\n%s', dtStr, ttl);
    end
    title(ttl);
    xlabel('Time (sec)'); ylabel('Error (arcsec)');
end
end

function plotFogmGrowth(t, fogm, sigma_ss, tau, figName, dtStr)
% Plot the FOGM ensemble growing from zero, overlaid with the theoretical
% growth envelope 3*sigma_ss*sqrt(1-exp(-2t/tau)) and the asymptotic 3-sigma bound.
% fogm: nSamp x 3 x nUnits [arcsec] ; sigma_ss: 1x3 [arcsec] steady-state 1-sigma
axisNames = {'X (cross)','Y (cross)','Z (boresight)'};
nUnits = size(fogm,3);
env = sqrt(1 - exp(-2*t(:)/tau));   % normalized growth: 0 -> 1
figure('Name',figName);
tiledlayout(3,1);
for ii = 1:3
    nexttile;
    plot(t, squeeze(fogm(:,ii,:)));
    hold on; grid on;
    hEnv = plot(t,  3*sigma_ss(ii)*env, 'm-.', 'LineWidth', 1.5);
    plot(t, -3*sigma_ss(ii)*env, 'm-.', 'LineWidth', 1.5);
    hSS = plot(t,  3*sigma_ss(ii)*ones(size(t)), 'k--', 'LineWidth', 1.2);
    plot(t, -3*sigma_ss(ii)*ones(size(t)), 'k--', 'LineWidth', 1.2);
    legend([hEnv hSS], '3\sigma_{ss}\surd(1-e^{-2t/\tau})', '3\sigma_{ss} steady-state', ...
        'Location','southeast');
    ttl = sprintf('FOGM Drift (from zero) - %s (%d units)', axisNames{ii}, nUnits);
    if ii == 1
        ttl = sprintf('%s\n%s', dtStr, ttl);
    end
    title(ttl);
    xlabel('Time (sec)'); ylabel('Error (arcsec)');
end
end

function plotBiasHistogram(biasAll, boundVec, figName, dtStr)
axisNames = {'X (cross)','Y (cross)','Z (boresight)'};
figure('Name',figName);
tiledlayout(3,1);
for ii = 1:3
    nexttile;
    histogram(biasAll(:,ii));
    hold on; grid on;
    yl = ylim;
    hb = plot([boundVec(ii) boundVec(ii)], yl, 'k--', 'LineWidth', 1.2);
    plot(-[boundVec(ii) boundVec(ii)], yl, 'k--', 'LineWidth', 1.2);
    legend(hb, '3-\sigma Bias Bound', 'Location','northeast');
    ttl = sprintf('Constant Bias - %s (%d units)', axisNames{ii}, size(biasAll,1));
    if ii == 1
        ttl = sprintf('%s\n%s', dtStr, ttl);
    end
    title(ttl);
    xlabel('Bias (arcsec)'); ylabel('Count');
end
end

function printStats(utIdx, neaErr, fogm, biasAll, nea_exp, fogm_exp, bias_exp)
% permute so the 3 sensor axes end up as columns before flattening samples/units
nea_emp  = std(reshape(permute(neaErr,[1 3 2]),[],3),0,1,'omitnan');
fogm_emp = std(reshape(permute(fogm,[1 3 2]),[],3),0,1,'omitnan');
bias_emp = std(biasAll,0,1,'omitnan');
fprintf('UT#%d 1-sigma summary [arcsec] (X cross, Y cross, Z boresight):\n', utIdx);
fprintf('  NEA  expected  : [% .3f % .3f % .3f]\n', nea_exp);
fprintf('  NEA  empirical : [% .3f % .3f % .3f]\n', nea_emp);
fprintf('  FOGM expected  : [% .3f % .3f % .3f]\n', fogm_exp);
fprintf('  FOGM empirical : [% .3f % .3f % .3f]\n', fogm_emp);
fprintf('  Bias expected  : [% .3f % .3f % .3f]\n', bias_exp);
fprintf('  Bias empirical : [% .3f % .3f % .3f]\n', bias_emp);
fprintf('\n');
end

function [lagTime, acfMean, acfTheory] = fogmAutocorr(fogm_3d, dt, tau, maxLagTime)
% Ensemble-averaged normalized autocorrelation via FFT.
% fogm_3d: nSamp x 3 x nUnits ; reshape keeps dim-1 (time), so each column
% of X is one axis/unit time series.
%
% The FOGM is zero-mean by construction, so we do NOT subtract the per-series
% sample mean: over a record only a few correlation times long, demeaning
% removes the slow drift and biases the autocorrelation sharply low (it would
% collapse to ~0 and cross zero near one time constant). We also use the
% unbiased (N-k) lag normalization instead of dividing by N.
[nSamp, ~, nUnits] = size(fogm_3d);
X = reshape(fogm_3d, nSamp, 3*nUnits);
X(isnan(X)) = 0;

maxLag = min(round(maxLagTime/dt), nSamp-1);
nfft = 2^nextpow2(2*nSamp);
F = fft(X, nfft);
S = real(ifft(abs(F).^2));          % S(k) = sum_t x(t) x(t+k), per column
S = S(1:maxLag+1, :);
nterms = (nSamp - (0:maxLag)).';    % overlapping-pair count at each lag
gammaHat = S ./ nterms;             % per-column unbiased autocovariance (mean known = 0)
% Pool across the axis/unit series BEFORE normalizing. Averaging per-series
% normalized ratios leaves the O(1/N_eff) finite-sample downward bias intact;
% normalizing the ensemble-averaged autocovariance drives that bias to
% ~O(1/(N_eff*M)) (M = number of series), so acfMean(k) ~ exp(-k*dt/tau).
gammaBar = mean(gammaHat, 2);       % ensemble-average autocovariance
acfMean  = gammaBar ./ gammaBar(1); % normalize pooled autocovariance by pooled variance
lagTime = (0:maxLag).' * dt;
acfTheory = exp(-lagTime/tau);
end
