classdef Pendulum
    %PENDULUM class contains all the identified constants for the
    %AeroShield pendulum system, with function to create various models
    %directly.

    properties
        L = 0.105; % m
        R = 0.0175; % m
        % m1 = 0.0540; % kg
        % m2 = 0.0965; % kg
        m1 = 0.0001; % kg
        m2 = 0.0115; % kg
        g = 9.81; % m/s^2
        
        I_T = 0;    % Moment of inertia
        M_1 = 0;    % Mass "torque" (being used in the gravitational torque)
        G_1 = 0;    % Gravitational torque
        Ku = 1/4160; % The input koefficient to convert %PWM to [Nm]. % home coefficient
        % Ku = 1/3000;

        %% All the frictional coefficients
        % xi = 0.0003; % damping coefficient % lab
        xi = 0.00002; % damping coefficient % home
        mu = 0.000160; % coloumb friction coefficient

        ka = 0.00000145; % is the air resistance coefficient
        kr = 1000; % hyperbolic tangent smoothing in the drag equation

        hc = 10000; % tanh smoothing in the coulomb friction
        
        omega_brk = 100; % the breakaway angular velocity threshold
        
        omega_S = 0; % Stribeck angular velocity threshold
        omega_dry = 0; % dry angular velocity threshold

        tau_brk = 2.4038e-06; % 0.5% of the control input, where the system does not move until at least 2% is supplied.

        n = 2; % Number of states and the order of the differential eq. describing the system.
        m = 1; % The number of outputs of the system.
        r = 1; % The number of inputs of the system.
    end

    properties (Access = private)
        L_
        R_
        m1_
        m2_
        g_
        Ku_
        xi_
        mu_
        ka_
        kr_
        hc_
        omega_brk_
        tau_brk_
    end

    methods
        function obj = Pendulum(L,R,m1,m2,g,Ku,xi,mu,ka,kr,hc,w_brk,t_brk)
            arguments (Input)
                L = 0.105;
                R = 0.0175;
                m1 = 0.0001;
                m2 = 0.0115;
                g = 9.81;
                Ku = 1/4160;
                xi = 0.00002;
                mu = 0.000160;
                ka = 0.00000145;
                kr = 1000;
                hc = 10000;
                w_brk = 100;
                t_brk = 2.4038e-06;
            end

            obj.L = L;
            obj.R = R;
            obj.m1 = m1;
            obj.m2 = m2;
            obj.g = g;
            obj.Ku = Ku;
            obj.xi = xi;
            obj.mu = mu;
            obj.ka = ka;
            obj.kr = kr;
            obj.hc = hc;
            obj.omega_brk = w_brk;
            obj.tau_brk = t_brk;

            obj.L_ = obj.L;
            obj.R_ = obj.R;
            obj.m1_ = obj.m1;
            obj.m2_ = obj.m2;
            obj.g_ = obj.g;
            obj.Ku_ = obj.Ku;
            obj.xi_ = obj.xi;
            obj.mu_ = obj.mu;
            obj.ka_ = obj.ka;
            obj.kr_ = obj.kr;
            obj.hc_ = obj.hc;
            obj.omega_brk_ = obj.omega_brk;
            obj.tau_brk_ = obj.tau_brk;

            obj = obj.updateCalcParameters();
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

        function [obj] = sett_brk(obj, t_brk, isscaling)
            arguments (Input)
                obj 
                t_brk 
                isscaling = false;
            end
            
            if isscaling
                obj.tau_brk = obj.tau_brk_ * t_brk;
            else
                obj.tau_brk = t_brk;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setw_brk(obj, w_brk, isscaling)
            arguments (Input)
                obj;
                w_brk;
                isscaling = false;
            end
            
            if isscaling
                obj.omega_brk = obj.omega_brk_ * w_brk;
            else
                obj.omega_brk = w_brk;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = sethc(obj, hc, isscaling)
            arguments (Input)
                obj;
                hc;
                isscaling = false;
            end
            
            if isscaling
                obj.hc = obj.hc_ * hc;
            else
                obj.hc = hc;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setkr(obj, kr, isscaling)
            arguments (Input)
                obj;
                kr;
                isscaling = false;
            end
            
            if isscaling
                obj.kr = obj.kr_ * kr;
            else
                obj.kr = kr;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setka(obj, ka, isscaling)
            arguments (Input)
                obj;
                ka;
                isscaling = false;
            end
            
            if isscaling
                obj.ka = obj.ka_ * ka;
            else
                obj.ka = ka;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setmu(obj, mu, isscaling)
            arguments (Input)
                obj;
                mu;
                isscaling = false;
            end
            
            if isscaling
                obj.mu = obj.mu_ * mu;
            else
                obj.mu = mu;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setxi(obj, xi, isscaling)
            arguments (Input)
                obj;
                xi;
                isscaling = false;
            end
            
            if isscaling
                obj.xi = obj.xi_ * xi;
            else
                obj.xi = xi;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setKu(obj, Ku, isscaling)
            arguments (Input)
                obj;
                Ku;
                isscaling = false;
            end
            
            if isscaling
                obj.Ku = obj.Ku_ * Ku;
            else
                obj.Ku = Ku;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setg(obj, g, isscaling)
            arguments (Input)
                obj;
                g;
                isscaling = false;
            end
            
            if isscaling
                obj.g = obj.g_ * g;
            else
                obj.g = g;
            end

            obj = obj.updateCalcParameters();
        end

        function [obj] = setm2(obj, m2)
            obj.m2 = m2;
            obj = obj.updateCalcParameters();
        end

        function [obj] = setm1(obj, m1)
            obj.m1 = m1;
            obj = obj.updateCalcParameters();
        end

        function [obj] = setR(obj, R)
            obj.R = R;
            obj = obj.updateCalcParameters();
        end

        function [obj] = setL(obj, L)
            obj.L = L;
            obj = obj.updateCalcParameters();
        end

        function [s] = tostring(obj)
            s = sprintf("g = %f, I_T = %f, M_1 = %f, G_1 = %f", obj.g, obj.I_T, obj.M_1, obj.G_1);
        end

        function print(obj)
            fprintf('%s\n', obj.tostring());
        end
    end

    methods (Access=private)
        function [obj] = updateCalcParameters(obj)
            obj.omega_S = obj.omega_brk*sqrt(2);
            obj.omega_dry = obj.omega_brk/10;
            % obj.tau_brk = obj.Ku/100;
            
            obj.I_T = 1/3*obj.m1*obj.L^2 + 2/5*obj.m2*obj.R^2 + obj.m2*(obj.L + obj.R)^2;
            obj.M_1 = obj.m1*obj.L/2 + obj.m2*(obj.L+obj.R);
            obj.G_1 = obj.g*obj.M_1;
        end
    end
end