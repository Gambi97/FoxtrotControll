tic; clearvars; close all; %clc;
addpath Classes\ P_code\ Functions\
load modelli.mat
load parametri_genetico_continuo.mat
load K_optim.mat
%% carico sistema e inizializzo controllore
[system, cs, Ts, u_max] = foxtrotInit;
P = zpk(modello_continuo);

%% ricavo parametri C1 e C2 dal control system designer
% s = tf("s");
Tf=0.13; % filtro sul setpoint
% F = 1/(Tf*s+1);
% 
% G1 = modello_continuo;    % fdt anello interno
% G2 = 1/s;                 % fdt anello esterno
% [Kp1, Ki1, Kd1, Tf1] = estrai_parametri(C1);
% [Kp2, Ki2, Kd2, Tf2] = estrai_parametri(C2);

%% inizializzo parametri ricavati dal tuning genetico
% Kp1 = x(1);
% Kp2 = x(2);
% Ki2 = x(3);
% Kd1 = x(4);
% Tf1 = x(5);
% Tf2 = x(6);
% Ki1 = 0;
% Kd2 = 0;

Kp1 = k(1);
Kp2 = k(2);
Ki2 = k(3);
Kd1 = k(4);
Tf1 = k(5);
Tf2 = k(6);
Ki1 = 0;
Kd2 = 0;
Tf = k(7);


fprintf("C1: Kp1 = %.2f, Ki1 = %.2f, Kd1 = %.2f, Tf1 = %.2f\n", Kp1, Ki1, Kd1, Tf1)
fprintf("C2: Kp2 = %.2f, Ki2 = %.2f, Kd2 = %.2f, Tf2 = %.2f\n", Kp2, Ki2, Kd2, Tf2)

%% controllore
% sovrascrivo il controllore del sistema con un controllore in cascata
ctrl = CascadeController(Ts, Tf, Kp1, Kp2, Ki1, Ki2, Kd1, Kd2, Tf1, Tf2, u_max);
ctrl.initialize;
cs.setController(ctrl);
%% 
t=(0:Ts:20)';

k = round(length(t)/2);
r = zeros(length(t),1);
r(1:k) = 3;
r(k+1:end) = 1;

cs.initialize
for idx=1:length(t)
    [y(idx,:), u(idx,1), t(idx,1)]=cs.step(r(idx));
end
pos = y(:,1);
vel = y(:,2);

% risposta del sistema senza errori di modello di evaluation (per
% velocizzare, poi si fa evaluation)
figure
plot(t, r, '--', t, pos)
axis([0 20 0 3.5]);
legend("riferimento", "risposta")
grid on

%% SCORE

[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)
figure(100)
subplot(2, 1, 1)
hold on
title("posizione")
% plottiamo limiti di tempo di assestamento (max 3 secondi al 2%)
plot([0 10], [2.94 2.94], 'r', [0 10], [3.06 3.06], 'r')
plot([10 20], [1.04 1.04], 'r', [10 20], [0.96 0.96], 'r')
plot([3 3], [0 5], 'r', [13 13], [0 5], 'r')
axis([0 20 0 3.5]);
xlabel("tempo")
ylabel("posizione")
hold off
subplot(2, 1, 2)
title("azione di controllo")
axis([0 20 -55 55]);
xlabel("tempo")
ylabel("coppia")

toc