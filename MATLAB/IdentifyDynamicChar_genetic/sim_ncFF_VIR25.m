% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku

function[t,y,dy,w,e,de,u,du]=sim_ncFF_VIR25(W1,W2,W3,Gs,r,t,x0,umin,umax,norms,noise,noise_amp)
% W1, W2, W3 = maice váh
% rezim = trenovanie / testovanie
dt = mean(diff(t));
nzeros = numel(Gs.num);

if(abs(Gs.num(1)) < 0.001)
    nzeros = nzeros - 1;
end

% FNORMS = "norms.csv";
% fhandle = fopen(FNORMS,"a+");
% write(fhandle, "Ny,Ndy,Ne,Nint,Nde,Ndu\n");

if nargin <= 7
    umax = 100;
    umin = -100;
end

if nargin<10 || isempty(norms)
    % defaultne normy
    y_max   = 220;           % ocakavane max(|y|)
    d1y_max = 50;           % ocakavane max(|dy/dt|)
    e_max   = 10;
    ie_max  = 10;    % odhad pre integral
    de_max = 100;
    d1u_max = 800;

    % y_max   = 1;           % ocakavane max(|y|)
    % d1y_max = 1;           % ocakavane max(|dy/dt|)
    % de_max = 1;
    % e_max   = 1;
    % ie_max  = 1;    % odhad pre integral
    % d1u_max = 1;

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

is_noise = false;
noise_K = 1;

if nargin >= 11
    is_noise = noise;
end
if nargin >= 12
    noise_K = noise_amp;
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

U_PB = 0;

for i=1:numel(t)
    e(i) = r(i) - ylast;
    de(i) = (e(i) - elast)/dt;
    eint = eint + e(i) * dt;

    % X=[ylast*Ny; dylast*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; dulast*Nd1u];
    X=[-dylast*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; dulast*Nd1u];
    % writenum2file(fhandle, X);
    X = max(min(X,1),-1); % orezanie na interval <-1,1>
    
    if size(W1, 2) ~= length(X)
        error('Nespravny pocet vstupov do NC podla vah W1 a vektora X.');
    end

    % NN control law
    A1=(W1*X);    % vstupna/1.skryta vrstva
    A1=tanh(3*A1);
    A2=(W2*A1);   % 1./2. skryta vrstva
    A2=tanh(3*A2);
    u(i)=W3*A2*umax;

    usat(i) = U_PB + min(umax, max(umin, u(i)));

    du(i) = (usat(i) - ulast)/dt;
    
    % Call the system differential
    ux = usat(i);
    if nzeros == 2
        ux = [usat(i), du(i)];
    end

    x = simtf(Gs, ux, dt, x);
    
    
    y(i) = x(1);
    dy(i) = x(2);

    x = x + flip(eye(size(x))*noise_K*randn(1))*is_noise;
    
    dulast = du(i);
    dylast = dy(i);
    ylast = y(i);
    elast = e(i);
    ulast = usat(i);
end
% fclose(fhandle);
end