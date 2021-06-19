clearvars; close all; %clc;
addpath Classes\ P_code\ Functions\
load modelli.mat
load parametri_genetico_continuo.mat
load K_optim.mat

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

%% controllore
% sovrascrivo il controllore del sistema con un controllore in cascata
ctrl = CascadeController(Ts, Tf, Kp1, Kp2, Ki1, Ki2, Kd1, Kd2, Tf1, Tf2, u_max);
ctrl.initialize;
cs.setController(ctrl);

score = [];
IAE =[];
OV =[];
CE =[];
settling_time= [];
for i=1:2
    
    [score(i), results]=cs.evalution;
    IAE(i)=max([results.IAE]);
    OV(i)=max([results.OV]);
    CE(i)=max([results.CE]);
    settling_time(i)=max([results.settling_time]);
end

parametri = struct("Score", score, "IAE", IAE, "CE", CE, "OV", OV, "ST",settling_time);
save robustezza parametri;