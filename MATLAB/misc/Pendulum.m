classdef Pendulum
    %PENDULUM class contains all the identified constants for the
    %AeroShield pendulum system, with function to create various models
    %directly.

    properties
        L = 0.1005; % m
        R = 0.0175; % m
        % m1 = 0.0540; % kg
        % m2 = 0.0965; % kg
        m1 = 0.003; % kg
        m2 = 0.012; % kg
        g = 9.81; % m/s^2
        
        I_T = 0;    % Moment of inertia
        M_1 = 0;    % Mass "torque" (being used in the gravitational torque)
        G_1 = 0;    % Gravitational torque
        Ku = 1/4163; % The input koefficient to convert %PWM to [Nm]. % home coefficient
        % Ku = 1/3000;

        %% All the frictional coefficients
        % xi = 0.0003; % damping coefficient % lab
        xi = 0.00005; % damping coefficient % home
        mu = 0.0000043; % coloumb friction coefficient

        ka = 0.0014; % is the air resistance coefficient
        kr = 0.001; % hyperbolic tangent smoothing in the drag equation

        hc = 10000; % tanh smoothing in the coulomb friction
        
        omega_brk = 0.0001; % the breakaway angular velocity threshold
        
        omega_S = 0; % Stribeck angular velocity threshold
        omega_dry = 0; % dry angular velocity threshold

        tau_brk = 0; % 0.5% of the control input, where the system does not move until at least 2% is supplied.

        n = 2; % Number of states and the order of the differential eq. describing the system.
        m = 1; % The number of outputs of the system.
        r = 1; % The number of inputs of the system.
    end

    methods
        function obj = Pendulum(g)
            arguments (Input)
                g = 9.81;
            end

            obj.g = g; % Assign gravitational acceleration to the object

            obj.omega_S = obj.omega_brk*sqrt(2);
            obj.omega_dry = obj.omega_brk/10;
            obj.tau_brk = obj.Ku/100;

            obj.I_T = 1/3*obj.m1*obj.L^2 + 2/5*obj.m2*obj.R^2 + obj.m2*(obj.L + obj.R)^2;
            obj.M_1 = obj.m1*obj.L/2 + obj.m2*(obj.L+obj.R);
            obj.G_1 = obj.g*obj.M_1;
        end

        function [A, B, C, D] = ss(obj)
            %ss Method is used for returning the model as a continuous
            %state-space model.
            [A, B, C, D] = create_ss_pendulum(obj);
        end

        function [A, B, C, D] = ss_discrete(obj, Ts)
            arguments (Input)
                obj;
                Ts = 0.01; % Sample time for discrete state-space model
            end
            [A, B, C, D] = create_ss_pendulum(obj, Ts);
        end

        function [obj, f, b, h, Fx, Bu, Hx] = nonlinear(obj, Ts, w_disturbance)
            arguments (Input)
                obj;
                Ts = 0.01; % Sample time for discrete state-space model
                w_disturbance = false; % Default disturbance value (in case we are trying to identify the deviation from real system)
            end
            if(w_disturbance)
                obj.n = 3;
            else
                obj.n = 2; % Decrease state count if no disturbance
            end
            [f, b, h, Fx, Bu, Hx] = create_nonlinear_pendulum(obj, Ts, w_disturbance);
        end

        function [s] = tostring(obj)
            s = sprintf("g = %f, I_T = %f, M_1 = %f, G_1 = %f", obj.g, obj.I_T, obj.M_1, obj.G_1);
        end

        function print(obj)
            fprintf('%s\n', obj.tostring());
        end
    end
end