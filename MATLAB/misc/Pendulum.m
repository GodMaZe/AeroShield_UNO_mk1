classdef Pendulum
    %PENDULUM class contains all the identified constants for the
    %AeroShield pendulum system, with function to create various models
    %directly.

    properties
        L = 0.1005; % m
        R = 0.0175; % m
        m1 = 0.0540; % kg
        m2 = 0.0965; % kg
        g = 9.81; % m/s^2
        xi = 0.0003; % damping coefficient
        mu = 0.00015; % coloumb friction coefficient
        I_T = 0;    % Moment of inertia
        M_1 = 0;    % Mass "torque" (being used in the gravitational torque)
        G_1 = 0;    % Gravitational torque
    end

    methods
        function obj = Pendulum(g)
            arguments (Input)
                g = 9.81;
            end

            obj.g = g; % Assign gravitational acceleration to the object

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

        function [f, h, Fx, Hx] = nonlinear(obj, Ts, use_saturation, w_disturbance)
            arguments (Input)
                obj;
                Ts = 0.01; % Sample time for discrete state-space model
                use_saturation = false; % Use saturation for coloumb friction or hyperbolic tangent.
                w_disturbance = false; % Default disturbance value (in case we are trying to identify the deviation from real system)
            end
            [f, h, Fx, Hx] = create_nonlinear_pendulum(obj, Ts, use_saturation, w_disturbance);
        end

        function [s] = tostring(obj)
            s = sprintf("g = %f, I_T = %f, M_1 = %f, G_1 = %f", obj.g, obj.I_T, obj.M_1, obj.G_1);
        end

        function print(obj)
            fprintf('%s\n', obj.tostring());
        end
    end
end