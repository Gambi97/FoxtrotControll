clearvars; close all; clc

addpath Classes\ P_code\ Functions\
load modelli.mat

%% carico sistema e inizializzo controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;

%% Controllo
Ki=0.6;
Kp=5;
Tf=0.1; % filtro sul setpoint

s = tf("s");
F = 1/(Tf*s+1);
C = Kp + Ki/s;
P = zpk(modello_continuo)*1/s;
% funzione di sensitività e/r = y/d
S = 1/(1+C*P);
% funzione di sensitività complementare y/r
Fc = C*P/(1+C*P);
% funzione di sensitività del controllo 1r
Q = C/(1+C*P);
% funzione d'anello
L = C*P;
% funzione di trasferimento y/r
G = F*feedback(L, 1);
ctrl=PIController(Ts,Ki,Kp,Tf);
cs.setController(ctrl);

t=(0:Ts:20)';
% r=ones(length(t),1);
k = round(length(t)/2);
r = zeros(length(t),1);
r(1:k) = 3;
r(k+1:end) = 1;


plot(t, r)
cs.initialize
for idx=1:length(t)
    [y(idx,:), u(idx,1), t(idx,1)]=cs.step(r(idx));
end
pos=y(:,1);
vel=y(:,2);
plot(t, r, t, pos)
[score,results]=cs.evalution;
IAE=max([results.IAE]);
OV=max([results.OV]);
CE=max([results.CE]);
settling_time=max([results.settling_time]);
fprintf('IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',IAE,OV,CE,settling_time,score)

