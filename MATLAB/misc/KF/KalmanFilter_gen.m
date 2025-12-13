classdef KalmanFilter_gen
    properties
        A   % state matrix (n x n)
        B   % input matrix (n x p)
        C   % output matrix (q x n)
        D   % feedthrough (q x p), optional
        G   % process noise input matrix (n x m), optional
        H   % direct noise-to-output matrix (q x m), optional

        Q   % process noise covariance (m x m) or (n x n) if G omitted
        R   % measurement noise covariance (q x q)
        N   % cross covariance between w and v (m x q), optional

        xhat    % current state estimate (n x 1)
        P       % current state covariance (n x n)

        steadyGain % logical: use steady-state filter
        Lsteady    % steady-state Kalman gain (n x q)
    end

    properties (Access = private)
        x0      % initial state
        P0      % initial covariance
    end

    methods
        function obj = KalmanFilter(A,B,C,varargin)
            % Required: A,B,C
            % Optional name-value pairs:
            % 'D', 'G', 'H', 'Q', 'R', 'N', 'x0', 'P0', 'Steady'
            p = inputParser;
            addRequired(p,'A');
            addRequired(p,'B');
            addRequired(p,'C');
            addParameter(p,'D',[0]);
            addParameter(p,'G',[]);
            addParameter(p,'H',[]);
            addParameter(p,'Q',[]);
            addParameter(p,'R',[]);
            addParameter(p,'N',[]);
            addParameter(p,'x0',[]);
            addParameter(p,'P0',[]);
            addParameter(p,'Steady',false);
            parse(p,A,B,C,varargin{:});

            obj.A = p.Results.A;
            obj.B = p.Results.B;
            obj.C = p.Results.C;
            obj.D = p.Results.D;
            obj.G = p.Results.G;
            obj.H = p.Results.H;
            obj.Q = p.Results.Q;
            obj.R = p.Results.R;
            obj.N = p.Results.N;
            obj.x0 = p.Results.x0;
            obj.P0 = p.Results.P0;
            obj.steadyGain = p.Results.Steady;

            n = size(obj.A,1);
            q = size(obj.C,1);

            if isempty(obj.x0)
                obj.x0 = zeros(n,1);
            end
            if isempty(obj.P0)
                obj.P0 = eye(n);
            end
            obj.xhat = obj.x0;
            obj.P = obj.P0;

            if isempty(obj.Q)
                % If G provided and Q not, use identity in noise space
                if ~isempty(obj.G)
                    m = size(obj.G,2);
                    obj.Q = eye(m);
                else
                    obj.Q = eye(n);
                end
            end
            if isempty(obj.R)
                obj.R = eye(q);
            end

            if obj.steadyGain
                obj = obj.computeSteadyGain();
            end
        end

        function obj = predict(obj,u)
            % Time update (predict)
            if nargin < 2, u = zeros(size(obj.B,2),1); end
            obj.xhat = obj.A*obj.xhat + obj.B*u;
            % process noise mapped into state space: use G*Q*G'
            if ~isempty(obj.G)
                Qx = obj.G * obj.Q * obj.G';
            else
                Qx = obj.Q;
            end
            obj.P = obj.A*obj.P*obj.A' + Qx;
        end

        function [obj, yhat] = update(obj,y,u)
            % Measurement update (correct)
            if nargin < 3, u = zeros(size(obj.B,2),1); end
            % predicted output (without measurement noise)
            ypred = obj.C*obj.xhat + (isempty(obj.D) && 0 || obj.D*u);
            S = obj.C*obj.P*obj.C' + obj.R;
            K = obj.P * obj.C' / S;    % Kalman gain
            innov = y - ypred;
            obj.xhat = obj.xhat + K * innov;
            obj.P = (eye(size(obj.P)) - K*obj.C) * obj.P;
            yhat = obj.C*obj.xhat + (isempty(obj.D) && 0 || obj.D*u);
            if obj.steadyGain
                % optionally override with steady gain to remove time variation
                obj.xhat = obj.xhat + (obj.Lsteady - K) * innov;
                obj.P = obj.P; % keep current P (user may want to keep evolving P)
            end
        end

        function [obj, yhat] = step(obj,u,y)
            % Convenience: predict then update
            if nargin < 2, u = zeros(size(obj.B,2),1); end
            if nargin < 3
                % if measurement not provided, perform predict-only
                obj = obj.predict(u);
                yhat = obj.C * obj.xhat + (isempty(obj.D) && 0 || obj.D*u);
                return
            end
            obj = obj.predict(u);
            [obj, yhat] = obj.update(y,u);
        end

        function obj = reset(obj)
            obj.xhat = obj.x0;
            obj.P = obj.P0;
        end

        function obj = computeSteadyGain(obj)
            % Compute steady-state Kalman gain for discrete-time using DARE
            % We form state-space with process noise G and measurement noise H:
            % Equivalent measurement noise covariance: Rbar = R + H*Q*H'
            % and effective Gbar = G
            if isempty(obj.G)
                % if G not provided, assume process enters state directly
                Gbar = eye(size(obj.A));
                Qbar = obj.Q;
            else
                Gbar = obj.G;
                Qbar = obj.Q;
            end
            if isempty(obj.H)
                Rbar = obj.R;
            else
                Rbar = obj.R + obj.H * obj.Q * obj.H';
            end

            % Solve discrete algebraic Riccati equation for estimation:
            % P = A*P*A' - A*P*C' * inv(C*P*C' + Rbar) * C*P*A' + Gbar*Qbar*Gbar'
            % Use built-in dare by converting to dual LQR: solve A'*S*A - ...
            % Equivalent dual: solve (A') S (A')' - ... with (C') as input matrix.
            try
                % Careful mapping: use dare on (A', C', Gbar*Qbar*Gbar', Rbar)
                S = dare(obj.A', obj.C', Gbar*Qbar*Gbar', Rbar);
                Pss = S;
                % steady Kalman gain L = P * C' * inv(C*P*C' + R)
                L = Pss * obj.C' / (obj.C*Pss*obj.C' + obj.R);
                obj.Lsteady = L;
                obj.steadyGain = true;
            catch
                warning('Failed to compute steady-state gain; leaving steadyGain=false.');
                obj.steadyGain = false;
            end
        end
    end
end
