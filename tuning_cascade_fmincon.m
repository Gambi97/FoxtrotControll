tic; clearvars; close all;
addpath Classes\ P_code\ Functions\
load modelli.mat
load x;

%% carico sistema e inizializzo il controllore
[system, cs, Ts, u_max] = foxtrotInit;
% Tf=0.1; % filtro sul setpoint

%% minimizzazione locale tramite fmincon della x trovata con il genetico più il filtro

J=@(k)f_Obj(k, cs, Ts, u_max);

opts = optimoptions(@fmincon);
opts.UseParallel = true;
opts.Display = 'iter';
opts.UseParallel = true;
lb = [0.01 0.01 0 0 0 0 0.01];
ub = [30 30 200 2 0.1 0.1 0.4];
k0 = [x, 0.1]; %Kp1, Kp2, Ki2, Kd1, Tf1, Tf2
problem = createOptimProblem('fmincon', 'x0', k0, 'objective', J, 'lb', lb, 'ub', ub);
[k, fval] = fmincon(J, k0, [], [], [], [], lb, ub, [], opts);

%%
ctrl=CascadeController(Ts, k(7), k(1), k(2), 0, k(3), k(4), 0, k(5), k(6), u_max);
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

toc



