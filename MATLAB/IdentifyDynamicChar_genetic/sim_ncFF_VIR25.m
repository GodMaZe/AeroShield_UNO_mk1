% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku

function[t,y,dy,w,e,de,u,usat,du,y_hat]=sim_ncFF_VIR25(W1,W2,W3,Gs,r,t,x0,umin,umax,norms,noise,noise_amp,A_tilde,B_tilde,C_tilde,Q,R,P,x_hat)
% W1, W2, W3 = maice váh
% rezim = trenovanie / testovanie
dt = mean(diff(t));
nzeros = numel(Gs.num);

if(abs(Gs.num(1)) < 0.001)
    nzeros = nzeros - 1;
end

% FNORMS = "norms.csv";
% fhandle = fopen(FNORMS,"a+");
% write(fhandle, "Np,Ny,Ndy,Nd,Ne,Nint,Nylast,Nd,Ndu\n");

if nargin <= 7
    umax = 100;
    umin = -100;
end

if nargin<10 || isempty(norms)
    % defaultne normy
    y_max   = 210;           % ocakavane max(|y|)
    d1y_max = 2408.8;           % ocakavane max(|dy/dt|)
    e_max   = 210;
    ie_max  = 2297.6;    % odhad pre integral
    de_max = 964.17;
    d1u_max = 4801.9;
    prop_max = 100;
    error_max = 45;
    u_max = umax;

    % y_max   = 1;           % ocakavane max(|y|)
    % d1y_max = 1;           % ocakavane max(|dy/dt|)
    % de_max = 1;
    % e_max   = 1;
    % ie_max  = 1;    % odhad pre integral
    % d1u_max = 1;
    % prop_max = 1;
    % error_max = 1;
    % u_max = 1;

    Ny=1/y_max; Nd1y=1/d1y_max;
    Ne=1/e_max; Nie=1/ie_max; Nd1u=1/d1u_max; Nde=1/de_max;
    Nprop=1/prop_max; Ner=1/error_max; Nu=1/u_max;
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
y_hat = zeros(size(t));
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

U_PB = 30;

max_X = zeros(9, 1);

for i=1:numel(t)
    e(i) = w(i) - ylast;
    de(i) = (e(i) - elast)/dt;
    eint = eint + e(i) * dt;

    
    % Do Kalman
    x_hat = A_tilde*x_hat + B_tilde*ulast;

    P = A_tilde*P*A_tilde' + Q;
    K = P*C_tilde'/(C_tilde*P*C_tilde' + R);
    e1 = ylast - C_tilde*x_hat;
    x_hat = x_hat + K*e1;
    y_hat(i) = C_tilde*x_hat;
    P = P - K*C_tilde*P;
    % End Kalman

    % X=[ylast*Ny; dylast*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; dulast*Nd1u];
    X=[x_hat(1)*Nprop; ylast*Ny; x_hat(3)*Nd1y; x_hat(4)*Ner; e(i)*Ne; eint*Nie; de(i)*Nde; ulast*Nu; dulast*Nd1u];
    max_X = max(X, max_X);
    % X=[-dylast*Nd1y; e(i)*Ne; eint*Nie; de(i)*Nde; dulast*Nd1u];
    % writenum2file(fhandle, X);
    % X = max(min(X,1),-1); % orezanie na interval <-1,1>

    
    if size(W1, 2) ~= length(X)
        error('Nespravny pocet vstupov do NC podla vah W1 a vektora X.');
    end

    % NN control law
    A1=(W1*X);    % vstupna/1.skryta vrstva
    A1=tanh(3*A1);
    A2=(W2*A1);   % 1./2. skryta vrstva
    A2=tanh(3*A2);
    ux=W3*A2*umax;
    u(i) = ulast + ux;

    usat(i) = min(umax, max(0, u(i)));
    
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
% disp(max_X);
% fclose(fhandle);
end