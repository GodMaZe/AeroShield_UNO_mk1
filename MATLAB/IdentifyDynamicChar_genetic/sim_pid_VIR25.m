% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku

function[t,y,dy,w,e,de,u,usat]=sim_pid_VIR25(P,I,D,Gs,r,t,x0,umin,umax,intmin,intmax)
% P, I, D = pid parametre
% rezim = trenovanie / testovanie
e = zeros(size(t));
de = zeros(size(t));
dy = zeros(size(t));
y = zeros(size(t));
u = zeros(size(t));
usat = zeros(size(t));
du = zeros(size(t));
w = r;

ulast = 0;
elast = 0;
eint = 0;

dt = mean(diff(t));

x = x0;

u_bottom = -inf;
u_top = inf;
antiwindup_min = -inf;
antiwindup_max = inf;

if nargin > 7
    u_bottom = umin;
end
if nargin > 8
    u_top = umax;
end
if nargin > 9
    antiwindup_min = intmin;
end
if nargin > 10
    antiwindup_max = intmax;
end

for i=1:numel(t)
    
    % PID control law
    u(i) = P * e(i) + I * eint + D * de(i);
    usat(i) = min(u_top, max(u_bottom, u(i)));

    du(i) = (usat(i) - ulast)/dt;
    
    % Call the system differential
    ux = usat(i);
    if numel(Gs.num) == 2
        ux = [usat(i), du(i)];
    end

    x = simtf(Gs, ux, dt, x);
    
    y(i) = x(1);
    
    if numel(x) > 1
        dy(i) = x(2);
    end

    e(i+1) = r(i) - x(1);
    de(i+1) = (e(i) - elast)/dt;
    eint = min(antiwindup_max, max(antiwindup_min, eint + e(i) * dt));

    elast = e(i);
    ulast = usat(i);
end
end