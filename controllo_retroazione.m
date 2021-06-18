tic
% clearvars;
close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat
% load controlli
%% carico sistema e inizializzo controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;
Tf=0.1; % filtro sul setpoint 
s = tf("s");
P = zpk(modello_continuo)*1/s; 
F = 1/(Tf*s + 1);
%% ricavo parametri C1 e C2
% C
% [Kp, Ki, Kd, N] = estrai_parametri(C);
Kp = 6;
Kd = 10;
Ki =  0;
N = 20;

fprintf("C1: Kp = %.2f, Ki = %.2f, Kd = %.2f, N = %.2f\n", Kp, Ki, Kd, N)

s = tf("s");
%controprova
C1 = Kp + Ki/s + N*Kd/(1+N/s);
% M = C1/C;
% minreal(M);
I = 1/s;

G = F*feedback(C1*P, 1);
H = 1;

%% creo il controllore in cascata
ctrl=PIDController_Vel_Filtered(Ts, Tf, Kp, Ki, Kd, N);
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

figure
plot(t, r, t, pos)
figure
step(G)
toc