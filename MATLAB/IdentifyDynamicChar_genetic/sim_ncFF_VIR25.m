% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku

function[t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(W1,W2,W3,Gs,r,t,x0,umax,umin,norms)
% W1, W2, W3 = maice váh
% rezim = trenovanie / testovanie

if nargin <= 7
    umax = 10;
    umin = 0;
end

if nargin<10 || isempty(norms)
    % defaultne normy
    y_max   = 40*8;           % ocakavane max(|y|)
    d1y_max = 7.16*10.5*1.4;           % ocakavane max(|dy/dt|)
    de_max = 2000;
    e_max   = 24*17.5;
    ie_max  = 1200;    % odhad pre integral
    d1u_max = 2000;

    Ny=1/y_max; Nd1y=1/d1y_max;
    Ne=1/e_max; Nie=1/ie_max; Nd1u=1/d1u_max; Nde=1/de_max;
else
    Ny   = norms.Ny;
    Nd1y = norms.Nd1y;
    Ne   = norms.Ne;
    Nie  = norms.Nie;
    Nde  = norms.Nde;
    Nd1u = norms.Nd1u;
end

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

for i=1:numel(t)
    e(i) = r(i) - x(1);
    de(i) = (e(i) - elast)/dt;
    eint = eint + e(i) * dt;

    X=[y(i)*Ny; dy(i)*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; du(i)*Nd1u]; % pripadne ine
    X = max(min(X,1),-1); % orezanie na interval <-1,1>
    
    if size(W1, 2) ~= length(X)
        error('Nespravny pocet vstupov do NC podla vah W1 a vektora X.');
    end

    % PID control law
    A1=(W1*X);    % vstupna/1.skryta vrstva
    A1=tanh(3*A1);
    A2=(W2*A1);   % 1./2. skryta vrstva
    A2=tanh(3*A2);
    u(i)=W3*A2*umax;

    usat(i) = min(umax, max(umin, u(i)));

    du(i) = (usat(i) - ulast)/dt;
    
    % Call the system differential
    ux = usat(i);
    if numel(Gs.num) == 2
        ux = [usat(i), du(i)];
    end

    x = simtf(Gs, ux, dt, x);
    
    
    y(i) = x(1);
    dy(i) = x(2);

    elast = e(i);
    ulast = usat(i);
end

end