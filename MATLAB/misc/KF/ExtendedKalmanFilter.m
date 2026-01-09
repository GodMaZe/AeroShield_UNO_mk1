classdef ExtendedKalmanFilter
    %EXTENDEDKALMANFILTER Class handling all the Extended Kalman Filter logic, simply create
    %the ExtendedKalmanFilter object as such:
    %
    %   ekf = ExtendedKalmanFilter(f,h,'Fx',Fx,'Hx',Hx,'Q',Q,'R',R,'x0',x0,'P0',P)
    % 
    % and within the measurement loop call the function:
    %
    %   [ekf, yhat] = ekf.step(u,y_measured)

    properties
        f   % state transition: x(k+1) = f(x, u)
        h   % measurement: y = h(x,u)
        Fx  % jacobian of the state transition function: df/dx (opt.)
        Hx  % jacobian of the measurement function: dh/dx (opt.)

        Q   % process noise covariance matrix (n x n) (n = number of states, order of the system)
        R   % measurement noise covariance matrix (m x m) (m = number of outputs)

        xhat    % current state estimate (n x 1)
        P   % current state covariance estimate (n x n)

        epstol  % epsilon tolerance for the differentiation
    end

    properties (Access = private)
        x0  % The initially estimated state of the system
        P0  % The initially estimated state covariance matrix
        I   % The identity matrix used for quicker matrix multiplication when updating the state covariance matrix at the end of the update function
        Zi  % A zero vector, used instead of the input vector, when non is passed into a function.
        is_JacobianState = false;    % Indicates whether the state transition Jacobian function was passed in while creating the EKF object.
        is_JacobianMeasurement = false; % Indicates whether the measurement Jacobian function was passed in while creating the EKF object.
        K   % Kalman gain matrix
        ninputs = 1;    % The number of inputs into the system
    end

    methods
        function obj = ExtendedKalmanFilter(f, h, x0, ninputs, varargin)
            arguments (Input)
                f;
                h;
                x0;
                ninputs = 1;
            end
            arguments (Input, Repeating)
                varargin;
            end
            arguments (Output)
                obj
            end
            
            p = inputParser;
            p.addRequired('f');
            p.addRequired('h');
            p.addRequired('x0');
            p.addRequired('ninputs');
            p.addParameter('Fx', []);
            p.addParameter('Hx', []);
            p.addParameter('Q', eye(size(x0)));
            p.addParameter('R', eye(size(h(0, x0, 0))));
            p.addParameter('P0', eye(size(x0)));
            p.addParameter('epstol', 1e-6);
            p.parse(f, h, x0, ninputs, varargin{:});

            obj.f = p.Results.f;
            obj.h = p.Results.h;
            obj.x0 = p.Results.x0;
            obj.ninputs = p.Results.ninputs;
            obj.Fx = p.Results.Fx;
            obj.is_JacobianState = ~isempty(obj.Fx);
            obj.Hx = p.Results.Hx;
            obj.is_JacobianMeasurement = ~isempty(obj.Hx);
            obj.Q = p.Results.Q;
            obj.R = p.Results.R;
            obj.P0 = p.Results.P0;
            obj.epstol = p.Results.epstol;
            obj.xhat = x0;
            obj.P = obj.P0;
            obj.I = eye(size(obj.P));
            obj.K = zeros(size(obj.R));
            obj.Zi = zeros(obj.ninputs, 1); % Zero input vector (placeholder for the input vector passed into the step function)
        end

        function [obj, yhat] = step(obj, t, u, y)
            if nargin < 3, u = obj.Zi; end
            obj = obj.predict(t, u);
            if nargin < 4 || isempty(y)
                yhat = obj.predict_output(t, u);
                fprintf("I am here!\n");
            else
                [obj, yhat] = obj.update(t, y, u);
            end
        end

        function K = getKalmanGain(obj)
            K = obj.K; % Return the current Kalman gain matrix
        end

        function obj = reset(obj)
            obj.xhat = obj.x0;
            obj.P = obj.P0;
        end
    end

    methods (Access = private)
        function obj = predict(obj, t, u)
            if nargin < 2, u = obj.Zi; end
            obj.xhat = obj.f(t, obj.xhat, u);
            if obj.is_JacobianState
                A = obj.Fx(t, obj.xhat, u);
            else
                A = obj.estimateNumericJacobian(@(x) obj.f(t, x, u), obj.xhat);
            end
            obj.P = A * obj.P * A' + obj.Q; % State covariance prediction
        end

        function ypred = predict_output(obj, t, u)
           ypred = obj.h(t, obj.xhat, u);
        end

        function [obj, yhat] = update(obj, t, y, u)
            assert(~isempty(y), "The feedback output cannot be left empty.");
            if nargin < 4, u = obj.Zi; end
            if obj.is_JacobianMeasurement
                C = obj.Hx(t, obj.xhat, u);
            else
                C = obj.estimateNumericJacobian(@(x) obj.h(t, x, u), obj.xhat);
            end
            % predicted output without correction
            ypred = obj.predict_output(t, u);
            % measurement
            S = C * obj.P * C' + obj.R;
            % correcting the Kalman Gain matrix
            obj.K = obj.P * C' / S;
            % error in the difference between the predicted output and
            % actual
            e1 = y - ypred;
            % state correction based on the output error
            obj.xhat = obj.xhat + obj.K * e1;
            % state covariance matrix correction
            % obj.P = obj.P - obj.K * C * obj.P;
            % Using the Joseph form for numerical stability, symmetry, and positive semidefiniteness preservation.
            obj.P = (obj.I - obj.K * C) * obj.P * (obj.I - obj.K * C)' + obj.K * obj.R * obj.K';
            obj.P = 0.5 * (obj.P + obj.P'); % Enforce exact symmetry
            yhat = obj.predict_output(t, u);
        end

        function J = estimateNumericJacobian(obj, h, x)
            % Calculate the numerical finite-difference Jacobian of f(x)
            fx = h(x);
            m = numel(fx);
            n = numel(x);
            J = zeros(m, n);
            for i = 1:n
                x1 = x;
                x1(i) = x1(i) + obj.epstol; % (x + h)
                fx1 = h(x1); % f(x + h)
                % df/dx = \lim_{h -> 0} \frac{f(x + h) - f(x)}{h}
                J(:, i) = (fx1 - fx) / obj.epstol;
            end
        end
    end
end