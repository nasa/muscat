classdef Software_SC_Orbit_Determination < handle
    
    properties
        % Filter
        config
        state_estimate
        covariance_estimate
    end
    
    methods
        function obj = Software_SC_Orbit_Determination(od_data, true_small_body, true_time)
            % State estimation configuration
            obj.config = [];
            obj.config.state = od_data.state; % 'absolute';
            obj.config.dynamics = od_data.dynamics; % 'keplerian';
            obj.config.predict_filter = od_data.predict_filter; % 'EKF';
            obj.config.update_filter  = od_data.update_filter;  % 'EKF';
            
            if strcmp(obj.config.state, 'absolute')
                obj.config.estimate_all = od_data.estimate_all; % true;
            end
            
            assert(ismember(obj.config.state, {'relative', 'absolute'}))
            assert(ismember(obj.config.dynamics, {'keplerian', 'HCW', 'rel_J2', 'grav_field'}))
            assert(ismember(obj.config.predict_filter, {'EKF', 'UKF'}))
            assert(ismember(obj.config.update_filter,  {'EKF', 'UKF'}))
            
            % Measurement type
            obj.config.use_gnss = true; % Absolute/relative position and velocity (GNSS)
            obj.config.use_range = false;
            obj.config.use_rangerate = false;
            obj.config.use_bearing_angles = false;
            
            % Measurement noise (giraloDistributedMultiGNSSTiming2019a)
            obj.config.sigma_pos_abs = 1.50e-3; % [km]   (1.5 m)
            obj.config.sigma_vel_abs = 0.03e-3; % [km/s] (0.04 m/s)
            obj.config.sigma_pos_rel = 3e-6; % [km]   (3 mm)
            obj.config.sigma_vel_rel = 1e-7; % [km/s] (0.1 mm/s)
            if obj.config.use_range
                obj.config.sigma_range = 1e-6; % [km]   (1 um)
            end
            if obj.config.use_rangerate
                obj.config.sigma_rangerate = 1e-7; % [km/s] (1 um/s)
            end
            if obj.config.use_bearing_angles
                obj.config.sigma_bearing_angles = deg2rad(1/3600); % [rad] (1 urad)
            end
            
            % Process noise
            dt = od_data.time_step;
            sigma_acc = 1e-5;
            Q_rv = sigma_acc^2 * eye(3);
            Q_i = [
                (1/3) * dt^4 * Q_rv, (1/2) * dt^3 * Q_rv;
                (1/2) * dt^3 * Q_rv,         dt^2 * Q_rv];
            obj.config.Q_i = Q_i;
            obj.config.dt = dt;
            
            % Inital uncertainty and process noise
            obj.config.sc_id = od_data.sc_id;
            if strcmp(obj.config.state, 'absolute')
                N_sc = size(od_data.rv_true,1);
                if obj.config.estimate_all
                    obj.config.N_state = 6 * N_sc;
                    obj.config.estimate_sat_ids = 1:N_sc;
                else
                    obj.config.N_state = 6;
                    obj.config.estimate_sat_ids = obj.config.sc_id;
                end
                sigma_pos_init = obj.config.sigma_pos_abs * 10;
                sigma_vel_init = obj.config.sigma_vel_abs * 10;
            elseif strcmp(obj.config.state, 'relative')
                N_deputy = size(od_data.rtn_true,1);
                obj.config.N_state = N_deputy * 6;
                obj.config.estimate_sat_ids = 1:N_deputy;
                sigma_pos_init = obj.config.sigma_pos_rel * 10;
                sigma_vel_init = obj.config.sigma_vel_rel * 10;
            end
            obj.config.N_prop = length(obj.config.estimate_sat_ids);
            
            
            % Initialize filter
            x_hat = zeros(obj.config.N_state, 1);
            P = zeros(obj.config.N_state, obj.config.N_state);
            P_i = diag([sigma_pos_init^2 * [1;1;1]; sigma_vel_init^2 * [1;1;1]]);
            for i_est = 1:obj.config.N_prop
                idx = 6*i_est-5:6*i_est;
                i_sc = obj.config.estimate_sat_ids(i_est);
                if strcmp(obj.config.state, 'absolute')
                    if obj.config.estimate_all
                        x_hat(idx) = od_data.rv_true(i_sc, :)' + mvnrnd(zeros(6,1), P_i)';
                    else
                        x_hat = od_data.rv_true(obj.config.sc_id, :)' + mvnrnd(zeros(6,1), P_i)';
                    end
                elseif strcmp(obj.config.state, 'relative')
                    x_hat(idx) = od_data.rtn_true(i_sc, :)' + mvnrnd(zeros(6,1), P_i)';
                    obj.config.rv_chief = od_data.rv_chief;
                end
                P(idx, idx) = P_i;
            end
            obj.state_estimate = x_hat;
            obj.covariance_estimate = P;
            
            % UKF parameters
            obj.config.sigma = 1e-3;
            obj.config.kappa = 1;
            obj.config.beta  = 2;
            
            % Dynamics setup
            mu = true_small_body.gravity_field.GM;
            obj.config.GM = mu;
            if strcmp(obj.config.dynamics, 'HCW')
                coe = osc2mean(Rv2Coe(obj.config.rv_chief, mu));
                obj.config.a = coe(1); % Semi-major axis [km]
            elseif strcmp(obj.config.dynamics, 'rel_J2')
                J2 = true_earth.gravity_field.J(2);
                Re = true_small_body.gravity_field.R;
                obj.config.k_J2 = 3 * J2 * mu * Re^2/2;
                obj.config.chief_param_ant = compute_chief_param_rel_J2(obj.config.rv_chief, mu, k_J2);
            elseif strcmp(obj.config.dynamics, 'grav_field')
                body_data.small_body_type = 6; % 6 = Earth
                body_data.gravity_degree_harmonics = 20;
                obj.config.est_time  = True_Time(time_data);
                obj.config.est_earth = True_Small_Body(body_data, true_time);
            end
        end
        
        function step(obj, measurement_available)
            % Retrieve data
            x_hat = obj.state_estimate;
            P = obj.covariance_estimate;
            N_state = obj.config.N_state;
            N_prop = obj.config.N_prop;
            mu = obj.config.GM;
            dt = obj.config.dt;
            
            % State, covariance, and STM
            x_hat_ = zeros(N_state, 1);
            P_ = zeros(N_state, N_state);
            
            if strcmp(obj.config.dynamics, 'HCW')
                coe = osc2mean(Rv2Coe(obj.config.rv_chief, mu));
                obj.config.a = coe(1); % Semi-major axis [km]
            elseif strcmp(obj.config.dynamics, 'rel_J2')
                u = zeros(3,1);
                chief_param = compute_chief_param_rel_J2(obj.config.rv_chief, mu, k_J2); % Change to estimate
                chief_params = [chief_param, chief_param_ant]';
            elseif strcmp(obj.config.dynamics, 'grav_field')
                u = zeros(3,1);
                est_time.func_set_time((ti-1) * dt);
                est_earth.func_update_SB_position_velocity_rot_matrix(est_time);
            end
            
            % ************** Predict **************
            for i_est = 1:N_prop
                idx = 1 + (i_est-1)*6 : i_est*6;
                % i_sc = estimate_sat_ids(i_est);
                % Spacecraft
                if strcmp(obj.config.state, 'absolute')
                    % R_rtn2eci = RotationEci2RtnRv(x_hat(idx))';
                    % Q = R_rtn2eci * Q_i * R_rtn2eci';
                    Q = obj.config.Q_i;
                elseif strcmp(obj.config.state, 'relative')
                    % Q = Q_i;
                    R_eci2rtn = RotationEci2RtnRv(x_hat(idx));
                    Q = R_eci2rtn * obj.config.Q_i * R_eci2rtn';
                end
                
                if strcmp(obj.config.predict_filter, 'EKF')
                    % ************** Extended Kalman Filter **************
                    if strcmp(obj.config.dynamics, 'keplerian')
                        [x_hat_i_, Phi] = propagate_kep_dynamics(x_hat(idx), mu, dt);
                    elseif strcmp(obj.config.dynamics, 'HCW')
                        [x_hat_i_, Phi] = propagate_hcw_dynamics(x_hat(idx), mu, dt, obj.config.a);
                    elseif strcmp(obj.config.dynamics, 'rel_J2')
                        [x_hat_i_, Phi] = propagate_rel_J2_dynamics(x_hat(idx), u, mu, dt, chief_params, obj.config.k_J2);
                    end
                    P_i_ = Phi * P(idx,idx) * Phi';
                    P_i_ = P_i_ + Q;
                    
                elseif strcmp(obj.config.predict_filter, 'UKF')
                    % ************** Unscented Kalman Filter **************
                    X_sigma = unscented_transform(x_hat(idx), P(idx,idx), sigma, kappa);
                    X_sigma_ = zeros(size(X_sigma));
                    for i_sigma = 1:size(X_sigma_,2)
                        if strcmp(obj.config.dynamics, 'keplerian')
                            X_sigma_(:, i_sigma) = propagate_kep_dynamics(X_sigma(:, i_sigma), mu, dt);
                        elseif strcmp(obj.config.dynamics, 'HCW')
                            X_sigma_(:, i_sigma) = propagate_hcw_dynamics(X_sigma(:, i_sigma), mu, dt, a);
                        elseif strcmp(obj.config.dynamics, 'rel_J2')
                            X_sigma_(:, i_sigma) = propagate_rel_J2_dynamics(X_sigma(:, i_sigma), u, mu, dt, chief_params, k_J2);
                        elseif strcmp(obj.config.dynamics, 'grav_field')
                            X_sigma_(:, i_sigma) = propagate_grav_field_dynamics(X_sigma(:, i_sigma), u, est_earth, dt);
                        end
                    end
                    [x_hat_i_, P_i_] = unscented_transform_inverse(X_sigma_, sigma, kappa);
                    P_i_ = P_i_ + Q;
                end
                
                assert(all(eig(P_i_) > 0), 'P is not positive definite')
                x_hat_(idx) = x_hat_i_;
                P_(idx, idx) = P_i_;
            end
            
            if strcmp(obj.config.dynamics, 'rel_J2')
                obj.config.chief_param_ant = chief_param;
            end
            
            % Combine measurements
            y_true_all = [];
            y_est_all_ = [];
            if strcmp(obj.config.update_filter, 'EKF')
                H_all = [];
                R_all = [];
            elseif strcmp(obj.config.update_filter, 'UKF')
                P_yy_all = [];
                P_xy_all = [];
            end
            
            % ************** Absolute State Measurement **************
            if (obj.config.use_gnss && strcmp(obj.config.state, 'absolute') && ~isempty(measurement_available.gnss_absolute)) || ...
                    (obj.config.use_gnss && strcmp(obj.config.state, 'relative') && ~isempty(measurement_available.gnss_relative))
                
                if strcmp(obj.config.state, 'absolute')
                    R_i = diag([obj.config.sigma_pos_abs^2 * ones(1,3), obj.config.sigma_vel_abs^2 * ones(1, 3)]);
                    y_true = measurement_available.gnss_absolute;
                    if ~obj.config.estimate_all
                        i_sat = obj.config.sc_id;
                        idx = 6*i_sat-5:6*i_sat;
                        y_true = y_true(idx);
                    end
                elseif strcmp(obj.config.state, 'relative')
                    R_i = diag([obj.config.sigma_pos_rel^2 * ones(1,3), obj.config.sigma_vel_rel^2 * ones(1, 3)]);
                    y_true = measurement_available.gnss_relative;
                end
                R = diag(repmat(diag(R_i), N_prop, 1));
                
                if strcmp(obj.config.update_filter, 'EKF')
                    % ************** Extended Kalman Filter **************
                    y_est_ = x_hat_;
                    H = eye(N_state);
                    
                    y_est_all_ = [y_est_all_; y_est_];
                    y_true_all = [y_true_all; y_true];
                    H_all = [H_all; H];
                    R_all = blkdiag(R_all, R);
                    
                elseif strcmp(obj.config.update_filter, 'UKF')
                    % ************** Unscented Kalman Filter **************
                    X_sigma = unscented_transform(x_hat_, P_, sigma, kappa);
                    Y_sigma = X_sigma;
                    [y_est_, P_xy, P_yy] = ...
                        unscented_transform_inverse_meas(x_hat_, X_sigma, Y_sigma, sigma, kappa, beta);
                    assert(all(eig(P_yy) > 0), 'P is not positive definite');
                    P_yy = P_yy + R;
                    assert(all(eig(P_yy) > 0), 'P is not positive definite');
                    assert(all(eig(P_xy) > 0), 'P is not positive definite');
                    
                    y_est_all_ = [y_est_all_; y_est_];
                    y_true_all = [y_true_all; y_true];
                    P_xy_all = [P_xy_all, P_xy];
                    P_yy_all = blkdiag(P_yy_all, P_yy);
                end
            end
            
            % ************** Range, Range Rate, and Bearing Angles Measurement **************
            if ((obj.config.use_range && ~isempty(measurement_available.range)) || ...
                    (obj.config.use_rangerate && ~isempty(measurement_available.rangerate)) || ...
                    (obj.config.use_bearing_angles && ~isempty(measurement_available.bearing_angles)))
                
                links = nchoosek(1:N_sc, 2);
                
                if strcmp(obj.config.state, 'absolute')
                    x_true = reshape(cell2mat(cellfun(@(rv) rv(ti,:), rv_true, 'UniformOutput', false))', [], 1);
                elseif strcmp(obj.config.state, 'relative')
                    x_true = reshape(cell2mat(cellfun(@(rtn) rtn(ti,:), rtn_true, 'UniformOutput', false))', [], 1);
                end
                y_true = compute_range_rangerate_bearing_angles(links, obj.config, x_true, true);
                
                if strcmp(obj.config.update_filter, 'EKF')
                    % ************** Extended Kalman Filter **************
                    [y_est_, H, R] = compute_range_rangerate_bearing_angles(links, obj.config, x_hat_, false);
                    
                    y_est_all_ = [y_est_all_; y_est_];
                    y_true_all = [y_true_all; y_true];
                    H_all = [H_all; H];
                    R_all = blkdiag(R_all, R);
                    
                elseif strcmp(obj.config.update_filter, 'UKF')
                    % ************** Unscented Kalman Filter **************
                    X_sigma = unscented_transform(x_hat_, P_, sigma, kappa);
                    N_meas = length(links) * ( ...
                        obj.config.use_range + obj.config.use_rangerate + 2*obj.config.use_bearing_angles);
                    Y_sigma = zeros(N_meas, size(X_sigma, 2));
                    for i_sigma = 1:size(X_sigma,2)
                        Y_sigma(:, i_sigma) = compute_range_rangerate_bearing_angles(links, obj.config, X_sigma(:, i_sigma), false);
                    end
                    [y_est_, P_xy, P_yy] = ...
                        unscented_transform_inverse_meas(x_hat_, X_sigma, Y_sigma, sigma, kappa, beta);
                    assert(all(eig(P_yy) > 0));
                    
                    y_est_all_ = [y_est_all_; y_est_];
                    y_true_all = [y_true_all; y_true];
                    P_xy_all = [P_xy_all, P_xy];
                    P_yy_all = blkdiag(P_yy_all, P_yy);
                end
            end
            
            % Measurement update
            if ~isempty(y_true_all)
                % Combine measurements
                dy = y_true_all - y_est_all_;
                
                if strcmp(obj.config.update_filter, 'EKF')
                    % ************** Extended Kalman Filter **************
                    H = H_all;
                    R = R_all;
                    
                    S = R + H * P_ * H';
                    K = P_ * H' / S;
                    J = eye(N_state) - K * H;
                    
                    x_hat = x_hat_ + K * dy;
                    P = J * P_ * J' + K * R * K';
                    
                elseif strcmp(obj.config.update_filter, 'UKF')
                    % ************** Unscented Kalman Filter **************
                    P_yy = P_yy_all;
                    P_xy = P_xy_all;
                    
                    K = P_xy / P_yy;
                    x_hat = x_hat_ + K * dy;
                    P = P_ - K * P_yy * K';
                end
                assert(all(eig(P) > 0), 'P is not positive definite')
            else
                % Only time update
                x_hat = x_hat_;
                P = P_;
            end
            
            % Store
            obj.state_estimate = x_hat;
            obj.covariance_estimate = P;
            
        end % end of step_filter
        
    end % end of methods
    
end % end of classdef
