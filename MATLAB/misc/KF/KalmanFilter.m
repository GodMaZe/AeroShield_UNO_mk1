classdef KalmanFilter
    %KALMANFILTER Class handling all the Kalman Filter logic, simply create
    %the KalmanFilter object as such:
    %
    %   kf = KalmanFilter(A,B,C,'Q',Q,'R',R,'x0',x0,'P0',P)
    % 
    % and within the measurement loop call the function:
    %
    %   [kf, yhat] = kf.step(u,y_measured)

    properties
        A   % system state matrix (n x n)
        B   % input matrix (n x r)
        C   % output matrix (m x n)
        D   % feedforward matrix (m x r), opt

        Q   % process noise covariance (n x n), opt
        R   % measurement noise covariance (m x m), opt

        xhat    % current state estimate (n x 1)
        P   % current state covariance matrix (n x n)
    end

    properties (Access = private)
        x0  % initial state
        P0  % initial state covariance
        I   % eye matrix (n x n) - substituting state covariance matrix
    end

    methods
        function obj = KalmanFilter(A, B, C, varargin)
            % Required: A, B, C
            % Optional name parameters:
            % 'Q', 'R', 'x0', 'P0'
            p = inputParser;
            addRequired(p, 'A');
            addRequired(p, 'B');
            addRequired(p, 'C');
            addParameter(p, 'D', zeros(size(C, 1), size(B, 2))); % default feedforward 
            addParameter(p, 'Q', eye(size(A))); % default process noise covariance
            addParameter(p, 'R', eye(size(C, 1))); % default measurement noise covariance
            addParameter(p, 'x0', zeros(size(A, 1), 1)); % default initial state
            addParameter(p, 'P0', eye(size(A))); % default initial covariance
            parse(p, A, B, C, varargin{:});

            obj.A = p.Results.A;
            obj.B = p.Results.B;
            obj.C = p.Results.C;
            obj.D = p.Results.D;
            obj.Q = p.Results.Q;
            obj.R = p.Results.R;
            obj.x0 = p.Results.x0;
            obj.P0 = p.Results.P0;
            obj.xhat = obj.x0;
            obj.P = obj.P0;
            obj.I = eye(size(obj.P));
        end

        function [obj, yhat] = step(obj, u, y)
            % Check whether u-param was entered, thus having 3 input
            % parameters
            if nargin < 2, u = zeros(size(obj.B, 2), 1); end
            obj = obj.predict(u);
            if nargin < 3
                yhat = obj.predict_output(u);
            else
                [obj, yhat] = obj.update(u, y);
            end
        end

        function obj = reset(obj)
            obj.xhat = obj.x0;
            obj.P = obj.P0;
        end

    end

    methods (Access = private)
        function obj = predict(obj, u)
            if nargin < 2, u = zeros(size(obj.B, 2), 1); end
            obj.xhat = obj.A * obj.xhat + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
        end

        function [obj, yhat] = update(obj, u, y)
            if nargin < 2, u = zeros(size(obj.B, 2), 1); end
            ypred = obj.predict_output(u);
            S = obj.C * obj.P * obj.C' + obj.R;
            K = (S'\(obj.C * obj.P'))';
            e1 = y - ypred;
            obj.xhat = obj.xhat + K * e1;
            obj.P = (obj.I - K * obj.C) * obj.P;
            % Using the Joseph form for numerical stability, symmetry, and positive semidefiniteness preservation.
            % obj.P = (obj.I - K * obj.C) * obj.P * (obj.I - K * obj.C) + K * obj.R * K';
            % obj.P = 0.5 * (obj.P + obj.P'); % Enforce exact symmetry
            yhat = obj.predict_output(u);
        end

        function ypred = predict_output(obj, u)
            ypred = obj.C * obj.xhat + obj.D * u;
        end
    end
end