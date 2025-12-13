classdef ExtendedKalmanFilter
    properties
        f           % state transition: x_next = f(x,u)
        h           % measurement: y = h(x,u)
        Fx           % function handle for df/dx (optional)
        Hx           % function handle for dh/dx (optional)

        Q           % process noise covariance (n x n)
        R           % measurement noise covariance (p x p)

        xhat        % current state estimate (n x 1)
        P           % current state covariance (n x n)

        x0          % initial state
        P0          % initial covariance

        epsFD = 1e-5 % finite-diff step (can be tuned)
    end

    methods
        function obj = ExtendedKalmanFilter(f,h,initialState,varargin)
            % obj = ExtendedKalmanFilter(f,h,initialState,Name,Value...)
            p = inputParser;
            addRequired(p,'f');
            addRequired(p,'h');
            addRequired(p,'initialState');
            addParameter(p,'Fx',[]);
            addParameter(p,'Hx',[]);
            addParameter(p,'Q',[]);
            addParameter(p,'R',[]);
            addParameter(p,'P0',[]);
            addParameter(p,'epsFD',obj.epsFD);
            parse(p,f,h,initialState,varargin{:});

            obj.f = p.Results.f;
            obj.h = p.Results.h;
            obj.Fx = p.Results.Fx;
            obj.Hx = p.Results.Hx;
            obj.x0 = p.Results.initialState(:);
            obj.xhat = obj.x0;
            obj.P0 = p.Results.P0;
            if isempty(obj.P0)
                n = numel(obj.x0);
                obj.P0 = eye(n);
            end
            obj.P = obj.P0;

            obj.Q = p.Results.Q;
            if isempty(obj.Q)
                n = numel(obj.x0);
                obj.Q = eye(n) * 1e-3;
            end
            obj.R = p.Results.R;
            if isempty(obj.R)
                % infer measurement size by calling h once (u = 0)
                try
                    y0 = obj.h(obj.x0, zeros(0,1));
                    pz = numel(y0);
                    obj.R = eye(pz) * 1e-2;
                catch
                    obj.R = eye(1) * 1e-2;
                end
            end

            obj.epsFD = p.Results.epsFD;
        end

        function obj = predict(obj,u)
            if nargin < 2, u = []; end
            % state prediction
            obj.xhat = obj.f(obj.xhat, u);
            % compute Fx (Jacobian df/dx)
            if isempty(obj.Fx)
                F = obj.numericJacobianX(@(x) obj.f(x,u), obj.xhat);
            else
                F = obj.Fx(obj.xhat, u);
            end
            % covariance prediction
            obj.P = F * obj.P * F' + obj.Q;
        end

        function [obj, ypred] = update(obj,y,u)
            if nargin < 3, u = []; end
            % compute H (Jacobian dh/dx)
            if isempty(obj.Hx)
                H = obj.numericJacobianX(@(x) obj.h(x,u), obj.xhat);
            else
                H = obj.Hx(obj.xhat, u);
            end
            % predicted measurement
            ypred = obj.h(obj.xhat, u);
            S = H * obj.P * H' + obj.R;
            K = obj.P * H' / S;
            innov = y(:) - ypred(:);
            obj.xhat = obj.xhat + K * innov;
            I = eye(size(obj.P));
            obj.P = (I - K * H) * obj.P * (I - K * H)' + K * obj.R * K'; % Joseph form
        end

        function [obj, ypred] = step(obj,u,y)
            % predict then update (if y provided)
            if nargin < 2, u = []; end
            obj = obj.predict(u);
            if nargin < 3 || isempty(y)
                ypred = obj.h(obj.xhat, u);
                return
            end
            [obj, ypred] = obj.update(y,u);
        end

        function obj = reset(obj)
            obj.xhat = obj.x0;
            obj.P = obj.P0;
        end
    end

    methods (Access = private)
        function J = numericJacobianX(obj,fun,x)
            % finite-difference Jacobian wrt x: fun returns column vector y
            fx = fun(x);
            m = numel(fx);
            n = numel(x);
            J = zeros(m,n);
            eps = obj.epsFD;
            for k = 1:n
                dx = zeros(n,1);
                dx(k) = eps * max(1,abs(x(k)));
                y1 = fun(x + dx);
                y0 = fx;
                J(:,k) = (y1(:) - y0(:)) ./ dx(k);
            end
        end
    end
end
