classdef stepwise < handle
    properties
        x       % State vector                      [x; y; yaw]
        P       % Covariance matrix                 [3x3]
        R       % Measurement noise covariance      [3x3]

        current_v_est           % Current estimated forward velocity
        gyro_bias_z             % Bias for Z-axis gyroscope
        accel_bias_fwd          % Bias for forward accelerometer
        w_pos                   % Std dev for position process noise component
        w_yaw_rate              % Std dev for yaw rate process noise component (rad/s)
        w_accel                 % Std dev for acceleration process noise component

        % --- Arena and Sensor Geometry ---
        arena_dim
        tof_rel_pos             % [[x1;y1], [x2;y2], [x3;y3]] relative offsets
        tof_rel_angle           % [theta1, theta2, theta3] relative angles
        epsilon
    end

    methods
        function obj = stepwise(initial_state, initial_covariance, predict_params, measurement_noise_r_std)
            % Constructor
            obj.x = double(initial_state(:));
            obj.P = double(initial_covariance);
            obj.current_v_est = 0.0;
            obj.gyro_bias_z = predict_params(1);
            obj.accel_bias_fwd = predict_params(2);
            obj.w_pos = predict_params(3);
            obj.w_yaw_rate = predict_params(4);
            obj.w_accel = predict_params(5);

            % --- Initialize Measurement Noise Covariance R ---
             if length(measurement_noise_r_std) == 3
                obj.R = diag(double(measurement_noise_r_std(:)).^2);
             else
                % If only one std dev provided, assume all ToF are the same
                if isscalar(measurement_noise_r_std)
                    obj.R = diag(repmat(double(measurement_noise_r_std)^2, 3, 1));
                    warning('Single measurement_noise_r_std provided; assuming identical noise for all 3 ToF sensors.');
                else
                     error('measurement_noise_r_std must have 1 or 3 elements');
                end
             end

            % --- Arena and Sensor Geometry ---
            W = 2.4; H = 2.4;
            obj.arena_dim = struct(...
                'x_min', -W/2.0, ...
                'x_max',  W/ 2.0, ...
                'y_min', -H/ 2.0, ...
                'y_max',  H/ 2.0 ...
            );
            obj.tof_rel_pos = double([
                 0.0173,  -0.2663, -0.0173; % x-offsets relative to robot center
                 0.0,  0.01,  0.01  % y-offsets relative to robot center
            ]);
            obj.tof_rel_angle = double([
                pi/2, pi, -pi/2 % Angle offsets relative to robot yaw
            ]);
            obj.epsilon = 1e-6; % Small number

            % disp('EKF Initialized (with myEKF Prediction Logic):');
            % fprintf('  Initial State (x): [%.3f, %.3f, %.3f rad]\n', obj.x(1), obj.x(2), obj.x(3));
            % disp('  Initial Covariance (P):'); disp(obj.P);
            % fprintf('  Gyro Bias Z: %.4f, Accel Bias Fwd: %.4f\n', obj.gyro_bias_z, obj.accel_bias_fwd);
            % fprintf('  Process Noise Std Devs (Pos:%.2f, YawRate:%.4f, Accel:%.2f)\n', obj.w_pos, obj.w_yaw_rate, obj.w_accel);
            % disp('  Measurement Noise (R):'); disp(obj.R);
        end
    end
end