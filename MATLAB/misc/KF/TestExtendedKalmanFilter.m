classdef TestExtendedKalmanFilter < matlab.unittest.TestCase
    methods (Test)
        function testEKFConverges(testCase)
            % System: x_k+1 = x_k + v (identity dynamics)
            % Measurement: z = x^2 + noise (nonlinear)
            dt = 1;
            N = 50;
            rng(0); % repeatable

            % True initial state
            x_true = 2.0;

            % Process and measurement noise
            q = 1e-4;   % process variance
            r = 0.1;    % measurement variance

            % stateTransitionFcn: identity
            stateTransitionFcn = @(x) x; % scalar

            % measurementFcn: nonlinear
            measurementFcn = @(x) x.^2;

            % Create EKF
            ekf = extendedKalmanFilter(stateTransitionFcn, measurementFcn, x_true + 0.5);
            ekf.ProcessNoise = q;
            ekf.MeasurementNoise = r;

            % Preallocate
            x_est = zeros(1,N);
            x_est(1) = ekf.State;

            % Simulate and filter
            x = x_true;
            errors = zeros(1,N);
            for k = 2:N
                % true process (no process disturbance for clarity)
                x = stateTransitionFcn(x);

                % measurement with noise
                z = measurementFcn(x) + sqrt(r) * randn;

                % EKF cycle
                predict(ekf);          % predict step
                correct(ekf, z);       % correct step

                x_est(k) = ekf.State;
                errors(k) = abs(x_est(k) - x);
            end

            % Verify error decreases (final error < initial error)
            testCase.verifyLessThanOrEqual(errors(end), errors(2), ...
                'EKF did not reduce error');

            % Verify final RMSE below threshold
            rmse = sqrt(mean((x_est - x_true).^2));
            testCase.verifyLessThan(rmse, 0.5, ...
                sprintf('Final RMSE too large: %.3f', rmse));
        end
    end
end
