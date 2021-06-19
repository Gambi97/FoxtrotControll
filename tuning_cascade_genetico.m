clearvars; close all; tic;
addpath Classes\ P_code\ Functions\
load modelli.mat
%%
[system, cs, Ts, u_max] = foxtrotInit;
Tf=0.12; % filtro sul setpoint

%% OTTIMIZZAZIONE CON ALGORITMO GENETICO
varNum = 6; % numero di variabili %Kp1, Kp2, Ki2, Kd1, Tf1, Tf2
lb = [0.01 0.01 0 0 0 0]; % lower bound
ub = [30 30 200 2 0.1 0.1]; % upper bound
Ad = []; % linear inequality constraints Ad*x <= Bd
Bd = []; % linear inequality constraints Ad*x <= Bd
Ae = []; % linear equality constraints Ae*x = Be
Be = []; % linear equality constraints Ae*x = Be
nonlinear = []; % nonlinear constraints
opts = optimoptions(@ga);
opts.UseParallel = true; 
opts.Display = 'iter';
opts.PlotFcn = 'gaplotbestindiv';

% funzione anonima per avere più ingressi nella funzione obiettivo
fit = @(x) f_Obj(x, cs, Ts, u_max, Tf);

[x, fval] = ga(fit, varNum, Ad, Bd, Ae, Be, lb, ub, nonlinear, opts);

toc