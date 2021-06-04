tic
clearvars; close all; %clc

addpath Classes\ P_code\ Functions\
load modelli.mat
load K2.mat
%% carico sistema e inizializzo controllore
system=Foxtrot;
Ts=system.getSamplingPeriod;
nInput = system.getInputNumber; % 1 input (coppia motore)
nOutput  = system.getOutputNumber; % 2 output (posizione, velocità)
cs=ControlledSystemFoxtrot(system);
cs.initialize; 
u_max = system.getUMax;

%% Controllo
Kp1 = 3.4+1;
Ki1 = 0;
Kd1 = 0;
Kp2 = 2.7;
Ki2 = 70;
Kd2 = 0.01;
N = 10;
Tf=0.1; % filtro sul setpoint 
fprintf("C1: Kp1 = %d, Ki1 = %d, Kd1 = %d\n", Kp1, Ki1, Kd1)
fprintf("C2: Kp2 = %d, Ki2 = %d, Kd2 = %d\n", Kp2, Ki2, Kd2)
ctrl=CascadeController(Ts, Tf, Kp1, Kp2, Ki1, Ki2, Kd1, Kd2, N);
cs.setController(ctrl);

t=(0:Ts:20)';
r=ones(length(t),1);
k = round(length(t)/2);
r = zeros(length(t),1);
r(1:k) = 3;
r(k+1:end) = 1;
cs.initialize;

for idx=1:length(t)
    [y(idx,:), u(idx,1), t(idx,1)]=cs.step(r(idx));
end
pos=y(:,1);
vel=y(:,2);
figure
plot(t, u)
figure
plot(t, r, t, pos)
hold on
plot([0 10], [2.94 2.94], [0 10], [3.06 3.06])
plot([10 20], [1.02 1.02], [10 20], [0.98 0.98])
plot([3 3], [0 5], [13 13], [0 5])
% score = zeros(1, 10);
% IAE = score;
% OV = score;
% CE = score;
% settling_time = score;
% for i = 1:1
%     [score(i),results]=cs.evalution;
%     IAE(i)=max([results.IAE]);
%     OV(i)=max([results.OV]);
%     CE(i)=max([results.CE]);
%     settling_time(i)=max([results.settling_time]);
%     fprintf('%d -> IAE=%f\tOV=%f\tCE=%f\tsettling_time=%f\tscore=%f\n',i,IAE(i),OV(i),CE(i),settling_time(i),score(i))
% end
% figure
% subplot(5, 1, 1)
% plot(score)
% title("SCORE")
% subplot(5, 1, 2)
% plot(IAE)
% title("IAE")
% subplot(5, 1, 3)
% plot(OV)
% title("OV")
% subplot(5, 1, 4)
% plot(CE)
% title("CE")
% subplot(5, 1, 5)
% plot(settling_time)
% title("settling_time")

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

% ctrl2=PIDController_Vel(Ts,Kp,Ti,Td,Tf);
% cs.setController(ctrl2);
% 
% t=(0:Ts:20)';
% % r=ones(length(t),1);
% k = round(length(t)/2);
% r = zeros(length(t),1);
% r(1:k) = 3;
% r(k+1:end) = 1;
% 
% 
% plot(t, r)
% cs.initialize
% for idx=1:length(t)
%     [y2(idx,:), u(idx,1), t(idx,1)]=cs.step(r(idx));
% end
% pos2=y2(:,1);
% vel2=y2(:,2);
% 
% figure
% plot(t, r, t, pos, t, pos2)
toc