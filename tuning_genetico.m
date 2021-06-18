clearvars; close all; tic;

addpath Classes\ P_code\ Functions\
load modelli.mat
%%
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;
Tf=0.1; % filtro sul setpoint

%%
varNum = 6; % numero di variabili %Kp1, Kp2, Ki2, Kd1, Tf1, Tf2
lb = [0.01 0.01 0 0 0 0]; % lower bound
ub = [30 30 200 2 0.1 0.1]; % upper bound
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
% opts.MaxTime = 7200; 
fit = @(x) f_Obj(x, cs, Ts, u_max, Tf);
[x, fval] = ga(fit, varNum, Ad, Bd, Ae, Be, lb, ub, nonlinear, opts);
toc