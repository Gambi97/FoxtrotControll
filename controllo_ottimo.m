%%
clearvars;close all;clc;
load modelli.mat

%% run ottimizazzione
wc_des=0.1;
J=@(x)pivel_cost_function(x,modello_continuo,wc_des);
%vincoli
MS=1.4;
wh=50;
Fh_max=0.1;
wl=0.01;
Dh_max=0.1;
PM_min=60;

w_vector=logspace(-3,3)'; % [10^-3,10^3]
w_vector=sort([w_vector;wl;wh]);
nlcon=@(x)pivel_constraints(x,modello_continuo,w_vector,MS,wh,Fh_max,wl,Dh_max,PM_min);

gs = GlobalSearch('Display','iter');
x0=[1,1];

problem = createOptimProblem('fmincon','x0',x0,...
    'objective',J,'lb',[0.01,0.01,0,0.01],'ub',[100,100,100,100],'nonlcon',nlcon);
x = run(gs,problem)

%%
addpath Classes\ Functions\ P_code\;
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;

Kp = x(1);
Ki = x(2);
Tf = 0.1; % filtro
ctrl = PIController(Ts, Ki, Kp, Tf);
cs.setController(ctrl);

[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)
figure(100)
subplot(2, 1, 1)
hold on
plot([0 10], [2.94 2.94], [0 10], [3.06 3.06])
plot([10 20], [1.02 1.02], [10 20], [0.98 0.98])
plot([3 3], [0 5], [13 13], [0 5])
