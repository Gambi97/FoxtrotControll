tic
clearvars;
close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat

%% carico sistema e inizializzo controllore
[system, cs, Ts, u_max] = foxtrotInit;

%% modello continuo 
s = tf("s"); %integratore per ottenere la posizione
I = 1/s;
Ps = modello_continuo*I;
As = Ps.A;
Bs = Ps.B;
Cs = Ps.C;
Ds = Ps.D;

%% modello discreto
Iz = c2d(I, Ts); 
Pz = modello_discreto*Iz;
Az = Pz.A;
Bz = Pz.B;
Cz = Pz.C;
Dz = Pz.D;
R = ctrb(Pz);
rank(R);
O = obsv(Pz);
rank(O);

%% controllore + stato aggiunto (errore) 

n=order(Pz);
no=size(Cz,1);
ni=size(Bz,2);

Ahat_z = [Az zeros(n,ni); -Cz*Ts eye(no,no)];
Bhat_z = [Bz; zeros(ni,1)];
Chat_z = [Cz 0; zeros(1,n) 1];
Dhat_z = zeros(no,ni);
sys_hat=ss(Ahat_z, Bhat_z, Chat_z, Dhat_z,Ts);

%% genetico
varNum = 4; % numero di variabili Qy(1), Qy(2), alpha, Tf
lb = [1 1 1.2, 0.01];
ub = [1000 1000 10 1];
Ad = []; % linear inequality constraints Ad*x <= Bd
% Ad(1, 1) = 1; Ad(1, 2) = -1;
Bd = []; % linear inequality constraints Ad*x <= Bd
Ae = []; % linear equality constraints Ae*x = Be
Be = []; % linear equality constraints Ae*x = Be
nonlinear = []; % nonlinear constraints
opts = optimoptions(@ga);
opts.UseParallel = true; 
opts.Display = 'iter';
opts.PlotFcn = 'gaplotbestindiv';
% opts.MaxGenerations = 500;
% opts.MaxTime = 36000; 
fit = @(x) f_Obj_LQR(x, cs, Ts, u_max, Az, Bz, Cz, sys_hat);
[x, fval] = ga(fit, varNum, Ad, Bd, Ae, Be, lb, ub, nonlinear, opts);
toc