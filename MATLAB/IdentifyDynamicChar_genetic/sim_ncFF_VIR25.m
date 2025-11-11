% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku

function[t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(W1,W2,W3,Gs,r,t,x0,umax,umin,norms)
% W1, W2, W3 = maice váh
% rezim = trenovanie / testovanie
dt = mean(diff(t));

FNORMS = "norms.csv";
fhandle = fopen(FNORMS,"a+");
% write(fhandle, "Ny,Ndy,Ne,Nint,Nde,Ndu\n");

if nargin <= 7
    umax = 100;
    umin = -100;
end

if nargin<10 || isempty(norms)
    % defaultne normy
    y_max   = 220/1.718;           % ocakavane max(|y|)
    d1y_max = 220/dt;           % ocakavane max(|dy/dt|)
    de_max = 1000/1.23;
    e_max   = 290;
    ie_max  = 4000/3.5/2.99;    % odhad pre integral
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

dylast = 0;
ylast= 0;
ulast = 0;
dulast = 0;
elast = 0;
eint = 0;



x = x0;

for i=1:numel(t)
    e(i) = r(i) - ylast;
    de(i) = (e(i) - elast)/dt;
    eint = eint + e(i) * dt;

    X=[ylast*Ny; dylast*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; dulast*Nd1u]; % pripadne ine
    writenum2file(fhandle, X);
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
    
    dulast = du(i);
    dylast = dy(i);
    ylast = y(i);
    elast = e(i);
    ulast = usat(i);
end
fclose(fhandle);
end