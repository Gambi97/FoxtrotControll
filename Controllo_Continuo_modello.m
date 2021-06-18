tic
% clearvars;
close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat
load X
%% carico sistema e inizializzo controllore
[system, cs, Ts, u_max] = foxtrotInit;
P = zpk(modello_continuo); 

%% ricavo parametri C1 e C2

s = tf("s");
Tf=0.13; % filtro sul setpoint
F = 1/(Tf*s+1);
G1 = modello_continuo;
G2 = 1/s;
% [Kp1, Ki1, Kd1, Tf1] = estrai_parametri(C1);
% [Kp2, Ki2, Kd2, Tf2] = estrai_parametri(C2);
Kp1 = x(1);
Kp2 = x(2);
Ki2 = x(3);
Kd1 = x(4);
Tf1 = x(5);
Tf2 = x(6);
Ki1 = 0;
Kd2 = 0;
fprintf("C1: Kp1 = %.2f, Ki1 = %.2f, Kd1 = %.2f, Tf1 = %.2f\n", Kp1, Ki1, Kd1, Tf1)
fprintf("C2: Kp2 = %.2f, Ki2 = %.2f, Kd2 = %.2f, Tf2 = %.2f\n", Kp2, Ki2, Kd2, Tf2)

C1 = 1/(Tf1*s+1)*(Kp1 + Ki1/s + Kd1*s);
C2 = 1/(Tf2*s+1)*(Kp2 + Ki2/s + Kd2*s);
L_int = C2*G1;
m_int = allmargin(L_int);
L_ext = C1*feedback(L_int, 1)*G2;
m_ext = allmargin(L_ext);

%%
ctrl = CascadeController(Ts, Tf, Kp1, Kp2, Ki1, Ki2, Kd1, Kd2, Tf1, Tf2, u_max);
ctrl.initialize;
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

toc