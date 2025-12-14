classdef ExtendedKalmanFilter
    %EXTENDEDKALMANFILTER Summary of this class goes here
    %   Detailed explanation goes here

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
    end

    methods
        function obj = ExtendedKalmanFilter(f, h, x0, varargin)
            arguments (Input)
                f;
                h;
                x0;
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
            p.addParameter('Fx', []);
            p.addParameter('Hx', []);
            p.addParameter('Q', eye(size(x0)));
            p.addParameter('R', numel(obj.h(obj.x0, 0)));
            p.addParameter('P0', eye(size(x0)));
            p.addParameter('epstol', 1e-3);
            p.parse(f, h, x0, varargin{:});

            obj.f = p.Results.f;
            obj.h = p.Results.h;
            obj.x0 = p.Results.x0;
            obj.Fx = p.Results.Fx;
            obj.Hx = p.Results.Hx;
            obj.Q = p.Results.Q;
            obj.R = p.Results.R;
            obj.P0 = p.Results.P0;
            obj.epstol = p.Results.epstol;
            obj.xhat = x0;
            obj.P = obj.P0;
        end

        function [obj, yhat] = step(obj, u, y)
            
        end

        function obj = reset(obj)
            obj.xhat = obj.x0;
            obj.P = obj.P0;
        end
    end

    methods (Access = private)
        function obj = predict(obj, u, y)
            
        end

        function obj = predict_output(obj, u, y)

        end
        function [obj, yhat] = update(obj, u, y)

        end
        function J = estimateNumericJacobian(obj, f, x)

        end
    end
end